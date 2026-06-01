"""Low-level controllers useful for vehicles.

The Lateral/Longitudinal PID controllers are adapted from `CARLA`_'s PID controllers,
which are licensed under the following terms:

    Copyright (c) 2018-2020 CVC.

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>.


.. _CARLA: https://carla.org/
"""

from abc import ABC, abstractmethod
from collections import deque
import math

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPoint, Point as ShapelyPoint

from scenic.core.regions import CircularRegion, PolylineRegion, toPolygon
from scenic.core.vectors import Vector


class LongitudinalController(ABC):
    @abstractmethod
    def computeThrottle(self):
        pass


class LateralController(ABC):
    def __init__(self):
        super().__init__()
        self.lastSteerAngle = None

    @abstractmethod
    def computeSteering(self, trajectory, obj):
        pass


class PIDController:
    def __init__(self, K_P, K_D, K_I, dt):
        super().__init__()
        self.kp = K_P
        self.ki = K_I
        self.kd = K_D
        self.dt = dt
        self.i_term = 0
        self.last_error = None
        self.windup_guard = 0.5 / self.ki if self.ki != 0 else 0

    def run_step(self, error):
        # Compute terms
        p_term = error
        self.i_term += error * self.dt
        self.i_term = np.clip(self.i_term, -self.windup_guard, self.windup_guard)
        d_term = (error - self.last_error) / self.dt if self.last_error else 0

        print(f"Error: {error}, LastError: {self.last_error}")
        # Remember last error for next calculation
        self.last_error = error

        output = (self.kp * p_term) + (self.ki * self.i_term) + (self.kd * d_term)
        clipped_output = np.clip(output, -1, 1)
        print(f"p_term: {p_term}")
        print(f"i_term: {self.i_term}")
        print(f"d_term: {d_term}")
        print(f"PID Output: {output}")
        print(f"PID Clipped Output: {clipped_output}")
        return clipped_output


class PIDLongitudinalController(PIDController):
    """Longitudinal control using a PID to reach a target speed.

    Arguments:
        K_P: Proportional gain
        K_D: Derivative gain
        K_I: Integral gain
        dt: time step
    """

    def __init__(self, K_P=0.5, K_D=0.1, K_I=0.2, dt=0.1):
        super().__init__(K_P, K_D, K_I, dt)


class PIDLateralController(PIDController, LateralController):
    """Lateral control using a PID to track a trajectory.

    Arguments:
        K_P: Proportional gain
        K_D: Derivative gain
        K_I: Integral gain
        dt: time step
    """

    def __init__(self, K_P=0.3, K_D=0.2, K_I=0, dt=0.1):
        super().__init__(K_P, K_D, K_I, dt)

    def computeSteering(self, trajectory, obj):
        assert isinstance(trajectory, PolylineRegion)
        cte = trajectory.signedDistanceTo(obj.position)
        # TODO: opposite traffic check?
        steer_angle = self.run_step(cte)
        self.lastSteerAngle = steer_angle
        return steer_angle


class PurePursuitLateralController(LateralController):
    def __init__(self, lookaheadDistance=lambda obj: obj.speed + 1):
        super().__init__()
        self.lookaheadDistance = lookaheadDistance
        self._lastTargetPoint = None

    def _findTargetPoint(self, trajectory, obj, lookaheadDistance):
        assert isinstance(trajectory, PolylineRegion)
        traj_line_string = toPolygon(trajectory)

        # Find candidate target points
        obj_pt = ShapelyPoint(*obj.position)
        obj_traj_dist = traj_line_string.project(obj_pt)
        forward_traj = shapely.ops.substring(
            traj_line_string, obj_traj_dist, traj_line_string.length
        )
        lookahead_circle = toPolygon(
            CircularRegion(forward_traj.coords[0], lookaheadDistance)
        )
        intersection_geometry = lookahead_circle.boundary.intersection(forward_traj)

        if intersection_geometry.is_empty:
            # No viable target points. If we have a last target point, aim for that. Otherwise,
            # aim for the closest point on the trajectory.
            if self._lastTargetPoint:
                target_point = self._lastTargetPoint
            else:
                target_point = trajectory.project(obj.position)
        elif isinstance(intersection_geometry, ShapelyPoint):
            target_point = intersection_geometry
        elif isinstance(intersection_geometry, MultiPoint):
            # There are multiple candidate target points. Pick the one that appears first on the path.
            target_point = sorted(
                intersection_geometry.geoms, key=lambda pt: shapely.distance(pt, obj_pt)
            )[0]
        else:
            # We've gotten something strange. Fall back to a representative point as the target.
            target_point = intersection_geometry.representative_point()

        # Store last target point and return
        self._lastTargetPoint = target_point
        return Vector(target_point.x, target_point.y)

    def computeSteering(self, trajectory, obj):
        # Compute target steering angle
        lookaheadDistance = self.lookaheadDistance(obj)
        targetPoint = self._findTargetPoint(trajectory, obj, lookaheadDistance)
        alpha = obj.angleTo(targetPoint) - obj.heading
        delta = -math.atan2(2 * obj.wheelbase * math.sin(alpha), lookaheadDistance)

        # Convert target steering angle to relative value in [-1, 1]
        rel_steering_angle = np.clip(delta / obj.maxSteeringAngle, -1, 1)

        # DEBUG
        print(f"SPEED: {obj.speed}")
        print(f"ALPHA: {delta}")
        print(f"DELTA: {rel_steering_angle}")
        print(f"RELATIVE STEERING ANGLE: {rel_steering_angle}")
        # import pygame
        # pygame.draw.circle(self.simulation.screen, (0,1,0), self.simulation.scenicToScreenVal(targetPoint), 5)
        # pygame.display.update()
        # import time
        # time.sleep(0.2)
        self.lastSteerAngle = rel_steering_angle
        return rel_steering_angle
