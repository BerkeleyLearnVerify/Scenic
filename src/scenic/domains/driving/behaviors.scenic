"""Library of useful behaviors for dynamic agents in driving scenarios.

These behaviors are automatically imported when using the driving domain.
"""

import math
import matplotlib.pyplot as plt
import csv

from scenic.domains.driving.actions import *
import scenic.domains.driving.model as _model
from scenic.domains.driving.roads import ManeuverType
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import LineString
from shapely.geometry import MultiPoint

def concatenateCenterlines(centerlines=[]):
    return PolylineRegion.unionAll(centerlines)

behavior ConstantThrottleBehavior(x):
    while True:
        take SetThrottleAction(x), SetReverseAction(False), SetHandBrakeAction(False)

behavior DriveAvoidingCollisions(target_speed=25, avoidance_threshold=10, pure_pursuit=False, plot=False):
    try:    
        do FollowLaneBehavior(target_speed=target_speed, purePursuit=pure_pursuit, plot=plot)
    interrupt when self.distanceToClosest(_model.Vehicle) <= avoidance_threshold:
        take SetThrottleAction(0), SetBrakeAction(1)

behavior AccelerateForwardBehavior():
    take SetReverseAction(False), SetHandBrakeAction(False), SetThrottleAction(0.5)

behavior WalkForwardBehavior():
    """Walk forward behavior for pedestrians.

    It will uniformly randomly choose either end of the sidewalk that the pedestrian is on, and have the pedestrian walk towards the endpoint.
    """
    current_sidewalk = _model.network.sidewalkAt(self.position)
    end_point = Uniform(*current_sidewalk.centerline.points)
    end_vec = end_point[0] @ end_point[1]
    normal_vec = Vector.normalized(end_vec)
    take WalkTowardsAction(goal_position=normal_vec), SetSpeedAction(speed=1)

behavior ConstantThrottleBehavior(x):
    take SetThrottleAction(x)

behavior FollowLaneBehavior(target_speed = 10, laneToFollow=None, is_oppositeTraffic=False, purePursuit=False, plot=False):
    """
    Follow's the lane on which the vehicle is at, unless the laneToFollow is specified.
    Once the vehicle reaches an intersection, by default, the vehicle will take the straight route.
    If straight route is not available, then any availble turn route will be taken, uniformly randomly.
    If turning at the intersection, the vehicle will slow down to make the turn, safely.

    This behavior does not terminate. A recommended use of the behavior is to accompany it with condition,
    e.g. do FollowLaneBehavior() until ...

    :param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
    :param laneToFollow: If the lane to follow is different from the lane that the vehicle is on, this parameter can be used to specify that lane. By default, this variable will be set to None, which means that the vehicle will follow the lane that it is currently on.
    """
    plotData = []
    
    past_steer_angle = 0
    past_speed = 0 # making an assumption here that the agent starts from zero speed
    if laneToFollow is None:
        current_lane = self.lane
    else:
        current_lane = laneToFollow

    current_centerline = current_lane.centerline
    in_turning_lane = False # assumption that the agent is not instantiated within a connecting lane
    intersection_passed = False
    entering_intersection = False # assumption that the agent is not instantiated within an intersection
    end_lane = None
    original_target_speed = target_speed
    TARGET_SPEED_FOR_TURNING = 5 # KM/H
    TRIGGER_DISTANCE_TO_SLOWDOWN = 10 # FOR TURNING AT INTERSECTIONS

    if current_lane.maneuvers != ():
        nearby_intersection = current_lane.maneuvers[0].intersection
        if nearby_intersection == None:
            nearby_intersection = current_lane.centerline[-1]
    else:
        nearby_intersection = current_lane.centerline[-1]

    # instantiate longitudinal and lateral controllers
    if(purePursuit):
        _lon_controller, _lat_controller = simulation().getPurePursuitControllers(self)
    else:
        _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)

    while True:

        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = past_speed

        if not entering_intersection and (distance from self.position to nearby_intersection) < TRIGGER_DISTANCE_TO_SLOWDOWN:
            entering_intersection = True
            intersection_passed = False
            straight_manuevers = filter(lambda i: i.type == ManeuverType.STRAIGHT, current_lane.maneuvers)

            if len(straight_manuevers) > 0:
                select_maneuver = Uniform(*straight_manuevers)
            else:
                if len(current_lane.maneuvers) > 0:
                    select_maneuver = Uniform(*current_lane.maneuvers)
                else:
                    take SetBrakeAction(1.0)
                    break

            # assumption: there always will be a maneuver
            if select_maneuver.connectingLane != None:
                current_centerline = concatenateCenterlines([current_centerline, select_maneuver.connectingLane.centerline, \
                    select_maneuver.endLane.centerline])
            else:
                current_centerline = concatenateCenterlines([current_centerline, select_maneuver.endLane.centerline])

            current_lane = select_maneuver.endLane
            end_lane = current_lane

            if current_lane.maneuvers != ():
                nearby_intersection = current_lane.maneuvers[0].intersection
                if nearby_intersection == None:
                    nearby_intersection = current_lane.centerline[-1]
            else:
                nearby_intersection = current_lane.centerline[-1]

            if select_maneuver.type != ManeuverType.STRAIGHT:
                in_turning_lane = True
                target_speed = TARGET_SPEED_FOR_TURNING

                do TurnBehavior(trajectory = current_centerline)


        if (end_lane is not None) and (self.position in end_lane) and not intersection_passed:
            intersection_passed = True
            in_turning_lane = False
            entering_intersection = False
            target_speed = original_target_speed

            if(purePursuit):
                _lon_controller, _lat_controller = simulation().getPurePursuitControllers(self)
            else:
                _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)


        if(not purePursuit):
            nearest_line_points = current_centerline.nearestSegmentTo(self.position)
            nearest_line_segment = PolylineRegion(nearest_line_points)
            cte = nearest_line_segment.signedDistanceTo(self.position)

            #Debugging prints
            #print(f"\n___Debugging___")
            #print(self.heading)
            #print(nearest_line_segment.orientation.)
        else:

            # Define variables
            lookahead_distance = _lat_controller.ld
            circlular_region = CircularRegion(self.position, lookahead_distance, resolution = 64)
            polyline_circle = circlular_region.boundary
            shapely_boundary = polyline_circle.lineString # extract shapley circle and shapely path
            line = current_centerline.lineString
            distance = current_centerline.lineString.project(ShapelyPoint(self.position.coordinates[0], self.position.coordinates[1]))
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
                print("No candidate point found")
                cte = 0
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
                
                theta = math.atan2(lookahead_point.y - self.position.coordinates[1], lookahead_point.x - self.position.coordinates[0])
                cte = (self.heading + math.pi/2) - theta


            # done


        if is_oppositeTraffic:
            cte = -cte

        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Lateral Control
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle
        past_speed = current_speed

        if plot:

            nearest_line_points = current_centerline.nearestSegmentTo(self.position) 
            nearest_line_segment = PolylineRegion(nearest_line_points)
            err = abs(nearest_line_segment.signedDistanceTo(self.position))
            plotData.append(err)

            """
            if len(plotData) == 125:
                with open('pid_err.csv', 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(plotData)
            """

            plt.plot(plotData)
            plt.ylabel('some numbers')
            plt.draw()
            plt.pause(0.001)
            plt.clf()


behavior FollowTrajectoryBehavior(target_speed = 10, trajectory = None, turn_speed=None):
    """
    Follows the given trajectory. The behavior terminates once the end of the trajectory is reached.

    :param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
    :param trajectory: It is a list of sequential lanes to track, from the lane that the vehicle is initially on to the lane it should end up on.
    """

    assert trajectory is not None
    assert isinstance(trajectory, list)
    if turn_speed is None:
        turn_speed = target_speed / 2

    brakeIntensity = 1.0
    distanceToEndpoint = 5 # meters

    traj_centerline = [traj.centerline for traj in trajectory]
    trajectory_centerline = concatenateCenterlines(traj_centerline)

    # instantiate longitudinal and lateral controllers
    _lon_controller,_lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0

    if trajectory[-1].maneuvers:
        end_intersection = trajectory[-1].maneuvers[0].intersection
        if end_intersection == None:
            end_intersection = trajectory[-1].centerline[-1]
    else:
        end_intersection = trajectory[-1].centerline[-1]

    while True:
        if self in _model.network.intersectionRegion:
            do TurnBehavior(trajectory_centerline, target_speed=turn_speed)

        if (distance from self to end_intersection) < distanceToEndpoint:
            break

        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        cte = trajectory_centerline.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle



behavior TurnBehavior(trajectory, target_speed=6):
    """
    This behavior uses a controller specifically tuned for turning at an intersection.
    This behavior is only operational within an intersection,
    it will terminate if the vehicle is outside of an intersection.
    """

    if isinstance(trajectory, PolylineRegion):
        trajectory_centerline = trajectory
    else:
        trajectory_centerline = concatenateCenterlines([traj.centerline for traj in trajectory])

    # instantiate longitudinal and lateral controllers
    _lon_controller, _lat_controller = simulation().getTurningControllers(self)

    past_steer_angle = 0

    while self in _model.network.intersectionRegion:
        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        cte = trajectory_centerline.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle


behavior LaneChangeBehavior(laneSectionToSwitch, is_oppositeTraffic=False, target_speed=10):

    """
    is_oppositeTraffic should be specified as True only if the laneSectionToSwitch to has
    the opposite traffic direction to the initial lane from which the vehicle started LaneChangeBehavior
    e.g. refer to the use of this flag in examples/carla/Carla_Challenge/carlaChallenge6.scenic
    """

    brakeIntensity = 1.0
    distanceToEndpoint = 3 # meters

    current_lane = laneSectionToSwitch.lane
    traj_centerline = [current_lane.centerline]
    trajectory_centerline = concatenateCenterlines(traj_centerline)

    if current_lane.maneuvers != ():
        nearby_intersection = current_lane.maneuvers[0].intersection
        if nearby_intersection == None:
            nearby_intersection = current_lane.centerline[-1]
    else:
        nearby_intersection = current_lane.centerline[-1]

    # instantiate longitudinal and lateral controllers
    _lon_controller, _lat_controller = simulation().getLaneChangingControllers(self)

    past_steer_angle = 0

    if not is_oppositeTraffic:
        traj_endpoint = current_lane.centerline[-1]
    else:
        traj_endpoint = current_lane.centerline[0]

    while True:
        if abs(trajectory_centerline.signedDistanceTo(self.position)) < 0.1:
            break
        if (distance from self to nearby_intersection) < distanceToEndpoint:
            straight_manuevers = filter(lambda i: i.type == ManeuverType.STRAIGHT, current_lane.maneuvers)

            if len(straight_manuevers) > 0:
                select_maneuver = Uniform(*straight_manuevers)
            else:
                if len(current_lane.maneuvers) > 0:
                    select_maneuver = Uniform(*current_lane.maneuvers)
                else:
                    take SetBrakeAction(1.0)
                    break

            # assumption: there always will be a maneuver
            if select_maneuver.connectingLane != None:
                trajectory_centerline = concatenateCenterlines([trajectory_centerline, select_maneuver.connectingLane.centerline, \
                    select_maneuver.endLane.centerline])
            else:
                trajectory_centerline = concatenateCenterlines([trajectory_centerline, select_maneuver.endLane.centerline])

            current_lane = select_maneuver.endLane

        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        cte = trajectory_centerline.signedDistanceTo(self.position)
        if is_oppositeTraffic: # [bypass] when crossing over the yellowline to opposite traffic lane
            cte = -cte

        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle
