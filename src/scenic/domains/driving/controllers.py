"""Low-level controllers useful for vehicles.

The Lateral/Longitudinal PID controllers are adapted from `CARLA`_'s PID controllers,
which are licensed under the following terms:

    Copyright (c) 2018-2020 CVC.

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>.


.. _CARLA: https://carla.org/
"""

from collections import deque
from math import sin

import numpy as np

from scenic.domains.driving.actions import *
from scenic.domains.driving.roads import ManeuverType
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import LineString
from shapely.geometry import MultiPoint
from scenic.core.regions import PolylineRegion
from scenic.core.regions import CircularRegion


class PIDLongitudinalController:
    """Longitudinal control using a PID to reach a target speed.

    Arguments:
        K_P: Proportional gain
        K_D: Derivative gain
        K_I: Integral gain
        dt: time step
    """

    def __init__(self, K_P=0.5, K_D=0.1, K_I=0.2, dt=0.1):
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, speed_error):
        """Estimate the throttle/brake of the vehicle based on the PID equations.

        Arguments:
            speed_error: target speed minus current speed

        Returns:
            a signal between -1 and 1, with negative values indicating braking.
        """
        error = speed_error
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip(
            (self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0
        )


class PIDLateralController:
    """Lateral control using a PID to track a trajectory.

    Arguments:
        K_P: Proportional gain
        K_D: Derivative gain
        K_I: Integral gain
        dt: time step
    """

    def __init__(self, K_P=0.3, K_D=0.2, K_I=0, dt=0.1):
        self.Kp = K_P
        self.Kd = K_D
        self.Ki = K_I
        self.PTerm = 0
        self.ITerm = 0
        self.DTerm = 0
        self.dt = dt
        self.last_error = 0
        self.windup_guard = 20.0
        self.output = 0

    def run_step(self, input_trajectory, ego, opposite_traffic):
        """Estimate the steering angle of the vehicle based on the PID equations.

        Arguments:
            cte: cross-track error (distance to right of desired trajectory)

        Returns:
            a signal between -1 and 1, with -1 meaning maximum steering to the left.
        """

        nearest_line_points = input_trajectory.nearestSegmentTo(ego.position)
        nearest_line_segment = PolylineRegion(nearest_line_points)
        cte = nearest_line_segment.signedDistanceTo(ego.position)

        if opposite_traffic:
            cte = -1 * cte
        
        error = cte
        delta_error = error - self.last_error
        self.PTerm = self.Kp * error
        self.ITerm += error * self.dt

        if self.ITerm < -self.windup_guard:
            self.ITerm = -self.windup_guard
        elif self.ITerm > self.windup_guard:
            self.ITerm = self.windup_guard

        self.DTerm = delta_error / self.dt

        # Remember last error for next calculation
        self.last_error = error

        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

        return np.clip(self.output, -1, 1)
    
class PurePursuitLateralController:
    """
    Pure Pursuit Controller

    Arguments:
        wb: wheelbase length
        ld: lookahead distance
        dt: time step
        cl: car length
        clbwr: car length to wheel base ratio
    """

    def __init__(self, cl = 4.5, ld = 7, dt = 0.1, clwbr = 0.65):
        """
        Todo:
            find the actual wheelbase and update the default number
                - it says car length is 4.5 meters, but im not sure if thats wheelbase
            experiment with the lookahead distance to see what works the best
        """
        self.dt = dt 
        self.wb = cl * clwbr
        self.ld = ld
        self.past_cte = 0

    def run_step(self, input_trajectory, ego, opposite_traffic):
        """Estimate the steering angle of the vehicle based on the pure pursuit equations.

        Arguments:
            cte: cross-track error (distance to right of desired trajectory)

        Returns:
            a signal between -1 and 1, with -1 meaning maximum steering to the left.

        Todo:
            Find if the equation needs to be updated to consider negative vs positive cte
            find what maximum steering distance is in radians and scale the return value accordingly
                -It is not listed in the documentation


            #takes current position and the path 
            #add plot 
        """
        # Define variables
        lookahead_distance = self.ld
        circlular_region = CircularRegion(ego.position, lookahead_distance, resolution = 64)
        polyline_circle = circlular_region.boundary
        shapely_boundary = polyline_circle.lineString # extract shapley circle and shapely path
        line = input_trajectory.lineString
        distance = input_trajectory.lineString.project(ShapelyPoint(ego.position.coordinates[0], ego.position.coordinates[1]))
        coords = []
        try: 
            coords = list(line.coords) # lineString case
        except NotImplementedError:
            for geom in line.geoms: # multilinestring case
                coords.extend(list(geom.coords))
        

        # Splitting the path into two parts, the part in front of the ego and behind it

        output = []
        for j, p in enumerate(coords):
            pd = line.project(ShapelyPoint(p[0], p[1]))
            if pd == distance:
                output = [
                    LineString(coords[:j+1]),
                    LineString(coords[j:])]
                break
            if pd > distance:
                cp = line.interpolate(distance)
                output = [
                    LineString(coords[:j] + [(cp.x, cp.y, 0)]),
                    LineString([(cp.x, cp.y, 0)] + coords[j:])]
                break


        # Get intersection points of the circle with the second half of the path

        shapely_intersection = shapely_boundary.intersection(output[1])
        candidate_points = []
        if isinstance(shapely_intersection, ShapelyPoint):
            candidate_points = [shapely_intersection]
        elif isinstance(shapely_intersection, MultiPoint):
            candidate_points = list(shapely_intersection.geoms)
        assert all(isinstance(p, ShapelyPoint) for p in candidate_points)


        # Find lookahead point by traversing the path

        if len(candidate_points) == 0:
            # print("No candidate point found") 
            # Sometimes no point is found, this is an error
            # In that case, dont touch steering wheel
            # Pure Pursuit is a self correcting algorithm so it should be fine
            cte = self.past_cte
        else:
            lookahead_point = candidate_points[0]
            best_distance = output[1].project(ShapelyPoint(candidate_points[0].x, candidate_points[0].y))
            if len(candidate_points) > 1:
                for point in candidate_points:
                    point_distance = output[1].project(ShapelyPoint(point.x, point.y))
                    if point_distance < distance:
                        lookahead_point = point
                        best_distance = point_distance


            # Find the theta value/cte to feed to the pure pursuit algorithm
            
            theta = math.atan2(lookahead_point.y - ego.position.coordinates[1], lookahead_point.x - ego.position.coordinates[0])
            cte = (self.heading + math.pi/2) - theta
            self.past_cte = cte
        
        if opposite_traffic:
            cte = -1 * cte

        # done

        rv = np.arctan((2 * self.wb * sin(cte)) / self.ld)

        return rv


        

