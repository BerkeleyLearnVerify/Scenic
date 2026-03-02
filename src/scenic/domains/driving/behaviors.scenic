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
import shapely.errors

def concatenateCenterlines(centerlines=[]):
    return PolylineRegion.unionAll(centerlines)

behavior ConstantThrottleBehavior(x):
    while True:
        take SetThrottleAction(x), SetReverseAction(False), SetHandBrakeAction(False)

behavior DriveAvoidingCollisions(lon_controller = None, lat_controller = None, target_speed=25, avoidance_threshold=10):
    _lon_controller = lon_controller
    _lat_controller = lat_controller
    try:    
        do FollowLaneBehavior(_lon_controller, _lat_controller, target_speed=target_speed)
    interrupt when self.distanceToClosest(_model.Vehicle) <= avoidance_threshold:
        take SetThrottleAction(0), SetBrakeAction(1)

behavior AccelerateForwardBehavior():
    take SetReverseAction(False), SetHandBrakeAction(False), SetThrottleAction(0.5)
    take SetThrottleAction(0), SetHandBrakeAction(False), SetThrottleAction(0.5)

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

behavior FollowLaneBehavior(target_speed = 10, lon_controller = None, lat_controller = None, laneToFollow=None, is_oppositeTraffic=False):
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

    
    # past_steer_angle = 0
    # past_speed = 0 # making an assumption here that the agent starts from zero speed
    if laneToFollow is None:
        current_lane = self.lane
    else:
        current_lane = laneToFollow

    do FollowTrajectoryBehavior(lon_controller = lon_controller, lat_controller = lat_controller, target_speed = target_speed, trajectory = [current_lane], is_oppositeTraffic = is_oppositeTraffic )

    


behavior FollowTrajectoryBehavior(lon_controller = None, lat_controller = None, target_speed = 10, trajectory = None, turn_speed=None, is_oppositeTraffic=False):
    """
    Follows the given trajectory. The behavior terminates once the end of the trajectory is reached.

    :param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
    :param trajectory: It is a list of sequential lanes to track, from the lane that the vehicle is initially on to the lane it should end up on.
    """

    assert trajectory is not None
    assert isinstance(trajectory, list)
    if turn_speed is None:
        turn_speed = target_speed / 2

    traj_centerline = [traj.centerline for traj in trajectory]
    trajectory_centerline = concatenateCenterlines(traj_centerline)

    in_turning_lane = False # assumption that the agent is not instantiated within a connecting lane
    intersection_passed = False
    entering_intersection = False # assumption that the agent is not instantiated within an intersection
    end_lane = None
    original_target_speed = target_speed
    TARGET_SPEED_FOR_TURNING = 5 # KM/H
    TRIGGER_DISTANCE_TO_SLOWDOWN = 10 # FOR TURNING AT INTERSECTIONS
    current_steer_angle, past_steer_angle = 0, 0

    #Instantiate default controllers if not specified
    if lon_controller is None or lat_controller is None:
        default_lon_controller, default_lat_controller = simulation().getPurePursuitControllers(self)

        if lon_controller is None:
            lon_controller = default_lon_controller

        if lat_controller is None:
            lat_controller = default_lat_controller

    
    current_lane = ego.lane

    if current_lane.maneuvers != ():
        nearby_intersection = current_lane.maneuvers[0].intersection
        if nearby_intersection == None:
            nearby_intersection = current_lane.centerline[-1]
    else:
        nearby_intersection = current_lane.centerline[-1]


    while True:

        current_lane = ego.lane

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
                trajectory_centerline = concatenateCenterlines([trajectory_centerline, select_maneuver.connectingLane.centerline, \
                    select_maneuver.endLane.centerline])
            else:
                trajectory_centerline = concatenateCenterlines([trajectory_centerline, select_maneuver.endLane.centerline])

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

                do TurnBehavior(trajectory = trajectory_centerline)


        if (end_lane is not None) and (self.position in end_lane) and not intersection_passed:
            intersection_passed = True
            in_turning_lane = False
            entering_intersection = False
            target_speed = original_target_speed

            if lon_controller is None or lat_controller is None:
                default_lon_controller, default_lat_controller = simulation().getPurePursuitControllers(self)

                if lon_controller is None:
                    lon_controller = default_lon_controller

                if lat_controller is None:
                    lat_controller = default_lat_controller


        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = lon_controller.run_step(speed_error)

        # compute steering : Lateral Control
        # Detect if the centerline has points in front of the car, use if statement

        
        if checkTrajectoryValidity(ego.position, trajectory_centerline):
            validTrajectory = trajectory_centerline
        else:
            validTrajectory = self.lane.centerline

        past_steer_angle = current_steer_angle
        current_steer_angle = lat_controller.run_step(validTrajectory, self, is_oppositeTraffic)
        

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle
        past_speed = current_speed


def checkTrajectoryValidity(position, trajectory):
    try:
        line = trajectory.lineString
        distance = trajectory.lineString.project(ShapelyPoint(position.coordinates[0], position.coordinates[1]))
        coords = []
        try: 
            coords = list(line.coords) # lineString case
        except NotImplementedError:
            for geom in line.geoms: # multilinestring case
                coords.extend(list(geom.coords))
        

        # Splitting the path into two parts, the part in front of the ego and behind it
        
        split_trajectory = []
        for j, p in enumerate(coords):
            pd = line.project(ShapelyPoint(p[0], p[1]))
            if pd == distance:
                split_trajectory = [
                    LineString(coords[:j+1]),
                    LineString(coords[j:])] #shapely.errors.GEOSException: IllegalArgumentException: point array must contain 0 or >1 elements
                break
            if pd > distance:
                cp = line.interpolate(distance)
                split_trajectory = [
                    LineString(coords[:j] + [(cp.x, cp.y, 0)]),
                    LineString([(cp.x, cp.y, 0)] + coords[j:])]
                break
        
        return True

    except shapely.errors.GEOSException:
        return False




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
    lon_controller, lat_controller = simulation().getTurningControllers(self)

    past_steer_angle = 0

    while self in _model.network.intersectionRegion:
        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        cte = trajectory_centerline.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = lat_controller.run_step(ego, False)

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
    lon_controller, lat_controller = simulation().getLaneChangingControllers(self)

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
        throttle = lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle


def getStraightestManeuver(maneuvers, currentLane):
    """Select the maneuver whose entry direction deviates least from the current lane.

    For each maneuver with a connecting lane, the heading deviation is the
    absolute angle between the current lane's final direction and the
    connecting lane's initial direction.  Direct merges (no connecting lane)
    are treated as perfectly straight (deviation = 0).
    """
    if len(maneuvers) == 1:
        return maneuvers[0]

    startDir = currentLane.centerline[-1] - currentLane.centerline[-2]

    def headingDeviation(maneuver):
        if maneuver.connectingLane is None:
            return 0
        entryDir = maneuver.connectingLane.centerline[1] - maneuver.connectingLane.centerline[0]
        return abs(startDir.angleWith(entryDir))

    return min(maneuvers, key=headingDeviation)


behavior FollowPolylineBehavior(polyline, target_speed=10, lon_controller=None, lat_controller=None):
    """
    Follows an arbitrary PolylineRegion, independent of the road network's
    lane structure.  Does not handle intersections — it simply tracks the
    given polyline using controllers.

    This behavior does not terminate.  A recommended usage pattern is:
        do FollowPolylineBehavior(polyline) until ...

    :param polyline: A PolylineRegion to follow
    :param target_speed: Target speed in m/s (default: 10)
    """

    assert isinstance(polyline, PolylineRegion)

    if lon_controller is None or lat_controller is None:
        default_lon, default_lat = simulation().getPurePursuitControllers(self)
        if lon_controller is None:
            lon_controller = default_lon
        if lat_controller is None:
            lat_controller = default_lat

    past_steer_angle = 0

    while True:
        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        speed_error = target_speed - current_speed
        throttle = lon_controller.run_step(speed_error)

        if checkTrajectoryValidity(self.position, polyline):
            current_steer_angle = lat_controller.run_step(polyline, self, False)
        else:
            current_steer_angle = past_steer_angle

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle


behavior FollowStraightestPathBehavior(target_speed=10, lon_controller=None, lat_controller=None):
    """
    Follows the current lane and at every intersection takes the straightest
    possible path, determined by the smallest heading deviation from the
    current lane direction.

    Unlike FollowTrajectoryBehavior which filters by ManeuverType.STRAIGHT
    and falls back to a random selection, this behavior computes the actual
    turn angle for every available maneuver and deterministically picks the
    one with the minimum heading change.

    This behavior does not terminate.

    :param target_speed: Target speed in m/s (default: 10)
    """

    current_lane = self.lane
    trajectory_centerline = concatenateCenterlines([current_lane.centerline])

    if lon_controller is None or lat_controller is None:
        default_lon, default_lat = simulation().getPurePursuitControllers(self)
        if lon_controller is None:
            lon_controller = default_lon
        if lat_controller is None:
            lat_controller = default_lat

    if current_lane.maneuvers != ():
        nearby_intersection = current_lane.maneuvers[0].intersection
        if nearby_intersection is None:
            nearby_intersection = current_lane.centerline[-1]
    else:
        nearby_intersection = current_lane.centerline[-1]

    entering_intersection = False
    intersection_passed = False
    end_lane = None
    original_target_speed = target_speed
    current_steer_angle, past_steer_angle = 0, 0
    TRIGGER_DISTANCE = 10

    while True:
        current_lane = self.lane

        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        if not entering_intersection and (distance from self.position to nearby_intersection) < TRIGGER_DISTANCE:
            entering_intersection = True
            intersection_passed = False

            if len(current_lane.maneuvers) == 0:
                take SetBrakeAction(1.0)
                break

            select_maneuver = getStraightestManeuver(current_lane.maneuvers, current_lane)

            if select_maneuver.connectingLane is not None:
                trajectory_centerline = concatenateCenterlines([
                    trajectory_centerline,
                    select_maneuver.connectingLane.centerline,
                    select_maneuver.endLane.centerline])
            else:
                trajectory_centerline = concatenateCenterlines([
                    trajectory_centerline,
                    select_maneuver.endLane.centerline])

            current_lane = select_maneuver.endLane
            end_lane = current_lane

            if current_lane.maneuvers != ():
                nearby_intersection = current_lane.maneuvers[0].intersection
                if nearby_intersection is None:
                    nearby_intersection = current_lane.centerline[-1]
            else:
                nearby_intersection = current_lane.centerline[-1]

            if select_maneuver.type != ManeuverType.STRAIGHT:
                target_speed = 5
                do TurnBehavior(trajectory=trajectory_centerline)

        if end_lane is not None and self.position in end_lane and not intersection_passed:
            intersection_passed = True
            entering_intersection = False
            target_speed = original_target_speed

        speed_error = target_speed - current_speed
        throttle = lon_controller.run_step(speed_error)

        if checkTrajectoryValidity(self.position, trajectory_centerline):
            validTrajectory = trajectory_centerline
        else:
            validTrajectory = self.lane.centerline

        past_steer_angle = current_steer_angle
        current_steer_angle = lat_controller.run_step(validTrajectory, self, False)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle
