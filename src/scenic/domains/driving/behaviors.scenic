"""Library of useful behaviors for dynamic agents in driving scenarios.

These behaviors are automatically imported when using the driving domain.
"""

import scenic.domains.driving.controllers as controllers
from scenic.domains.driving.actions import *
import scenic.domains.driving.model as _model
from scenic.simulators.carla.blueprints import *
from scenic.domains.driving.roads import ManeuverType
from scenic.core.regions import regionFromShapelyObject
from shapely.geometry import LineString
import math

def concatenateCenterlines(centerlines=[]):
    return PolylineRegion.unionAll(centerlines)

def setLaneFollowingPIDControllers(is_vehicle, dt):
    if is_vehicle:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0, dt=dt)

    else:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)

    return lon_controller, lat_controller

def setTurnPIDControllers(is_vehicle, dt):
    if is_vehicle:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.8, K_D=0.2, K_I=0, dt=dt)

    else:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.4, K_D=0.1, K_I=0.0, dt=dt)

    return lon_controller, lat_controller

def setLaneChangingPIDControllers(is_vehicle, dt):
    if is_vehicle:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.08, K_D=0.3, K_I=0, dt=dt)

    else:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.1, K_D=0.3, K_I=0.0, dt=dt)

    return lon_controller, lat_controller


behavior ConstantThrottleBehavior(x):
    while True:
        take SetThrottleAction(x), SetReverseAction(False), SetHandBrakeAction(False)

behavior DriveAvoidingCollisions(target_speed=25, avoidance_threshold=10):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
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

behavior FollowLaneBehavior(target_speed = 10, laneToFollow=None, is_oppositeTraffic=False):
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


    # check whether self agent is vehicle:
    if hasattr(self, 'blueprint'):
        is_vehicle = self.blueprint in carModels
    else:
        # assume it is a car
        is_vehicle = True

    dt = simulation().timestep
    
    # instantiate longitudinal and latitudinal pid controllers
    _lon_controller, _lat_controller = setLaneFollowingPIDControllers(is_vehicle, dt)

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
            _lon_controller, _lat_controller = setLaneFollowingPIDControllers(is_vehicle, dt)

        nearest_line_points = current_centerline.nearestSegmentTo(self.position)
        nearest_line_segment = PolylineRegion(nearest_line_points)
        cte = nearest_line_segment.signedDistanceTo(self.position)
        if is_oppositeTraffic:
            cte = -cte

        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle
        past_speed = current_speed


behavior FollowTrajectoryBehavior(target_speed = 10, trajectory = None):
    """ 
    Follows the given trajectory. The behavior terminates once the end of the trajectory is reached.

    :param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
    :param trajectory: It is a list of sequential lanes to track, from the lane that the vehicle is initially on to the lane it should end up on.  
    """

    assert trajectory is not None
    assert isinstance(trajectory, list)

    brakeIntensity = 1.0
    distanceToEndpoint = 5 # meters

    traj_centerline = [traj.centerline for traj in trajectory]
    trajectory_centerline = concatenateCenterlines(traj_centerline)

    # check whether self agent is vehicle:
    if hasattr(self, 'blueprint'):
        is_vehicle = self.blueprint in carModels
    else:
        # assume it is a car
        is_vehicle = True

    dt = simulation().timestep
    _lon_controller,_lat_controller = setLaneFollowingPIDControllers(is_vehicle, dt)

    # instantiate longitudinal and latitudinal pid controllers
    past_steer_angle = 0
    
    if trajectory[-1].maneuvers:
        end_intersection = trajectory[-1].maneuvers[0].intersection
        if end_intersection == None:
            end_intersection = trajectory[-1].centerline[-1]
    else:
        end_intersection = trajectory[-1].centerline[-1]

    while True:
        if self in _model.network.intersectionRegion:
            do TurnBehavior(trajectory_centerline)

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
    This behavior uses a PID controller specifically tuned for turning at an intersection. 
    This behavior is only operational within an intersection, 
    it will terminate if the vehicle is outside of an intersection. 
    """

    if isinstance(trajectory, PolylineRegion):
        trajectory_centerline = trajectory
    else:
        trajectory_centerline = concatenateCenterlines([traj.centerline for traj in trajectory])

    dt = simulation().timestep
    # check whether self agent is vehicle:
    if hasattr(self, 'blueprint'):
        is_vehicle = self.blueprint in carModels
    else:
        # assume it is a car
        is_vehicle = True

    # instantiate longitudinal and latitudinal pid controllers
    _lon_controller, _lat_controller = setTurnPIDControllers(is_vehicle, dt)

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

    # check whether self agent is vehicle:
    if hasattr(self, 'blueprint'):
        if (self.blueprint in carModels) or (self.blueprint in truckModels):
            is_vehicle = True
        else:
            is_vehicle = False
    else:
        # assume it is a car`
        is_vehicle = True

    dt = simulation().timestep

    # instantiate longitudinal and latitudinal pid controllers
    _lon_controller, _lat_controller = setLaneChangingPIDControllers(is_vehicle, dt)

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
