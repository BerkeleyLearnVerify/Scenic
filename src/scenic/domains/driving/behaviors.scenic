
import scenic.domains.driving.controllers as controllers
from scenic.domains.driving.actions import *
# import scenic.domains.driving.model as model
from scenic.domains.driving.model import Vehicle, network
from scenic.simulators.carla.blueprints import *
from scenic.domains.driving.roads import ManeuverType
from scenic.core.regions import regionFromShapelyObject
from shapely.geometry import LineString
import math

behavior ConstantThrottleBehavior(x):
    while True:
        take SetThrottleAction(x), SetReverseAction(False), SetHandBrakeAction(False)

behavior DriveAvoidingCollisions(target_speed=25, avoidance_threshold=10):
    try:
        FollowLaneBehavior(target_speed=target_speed)
    interrupt when self.distanceToClosest(Vehicle) <= avoidance_threshold:
        take SetThrottleAction(0), SetBrakeAction(1)

def concatenateCenterlines(centerlines=[]):
    return PolylineRegion.unionAll(centerlines)

def distance(pos1, pos2):
    """ pos1, pos2 = (x,y) """
    return math.sqrt(math.pow(pos1[0]-pos2[0],2) + math.pow(pos1[1]-pos2[1],2))

def distanceToAnyObjs(vehicle, thresholdDistance):
    """ checks whether there exists any obj
    (1) in front of the vehicle, (2) on the same lane, (3) within thresholdDistance """
    objects = simulation().objects
    for obj in objects:
        if not (vehicle can see obj):
            continue
        if not (network.laneAt(vehicle) == network.laneAt(obj) or network.intersectionAt(vehicle)==network.intersectionAt(obj)):
            continue
        if distance(vehicle.position, obj.position) < 0.1:
            # this means obj==vehicle
            pass
        elif distance(vehicle.position, obj.position) < thresholdDistance:
            return True
    return False

behavior AccelerateForwardBehavior():
    take SetReverseAction(False)
    take SetHandBrakeAction(False)
    take SetThrottleAction(0.5)

behavior WalkForwardBehavior():
    current_sidewalk = network.sidewalkAt(self.position)
    end_point = Uniform(*current_sidewalk.centerline.points)
    end_vec = end_point[0] @ end_point[1]
    normal_vec = Vector.normalized(end_vec)
    take WalkTowardsAction(goal_position=normal_vec)
    take SetSpeedAction(speed=1)

behavior ConstantThrottleBehavior(x):
    take SetThrottleAction(x)

behavior FollowLaneBehavior(target_speed = 10, laneToFollow=None):
    ## Follow's the lane on which the vehicle is at 
    ## As the vehicle reaches an intersection, any route (eg. straigth or turn maneuver) is randomly selected and followed
    
    past_steer_angle = 0
    past_speed = 0 # making an assumption here that the agent starts from zero speed
    if laneToFollow is None:
        current_lane = network.laneAt(self)
    else:
        current_lane = laneToFollow

    current_centerline = current_lane.centerline
    in_turning_lane = False # assumption that the agent is not instantiated within a connecting lane
    entering_intersection = False # assumption that the agent is not instantiated within an intersection
    end_lane = None
    original_target_speed = target_speed
    TARGET_SPEED_FOR_TURNING = 7 # KM/H
    TRIGGER_DISTANCE_TO_SLOWDOWN = 20 # FOR TURNING AT INTERSECTIONS
    nearby_intersection = current_lane.maneuvers[0].intersection

    # check whether self agent is vehicle:
    if self.blueprint:
        is_vehicle = self.blueprint in carModels
    else:
        # assume it is a car
        is_vehicle = True

    dt = simulation().timestep
    
    # instantiate longitudinal and latitudinal pid controllers
    if is_vehicle:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0, dt=dt)

    else:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)

    while True:

        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = past_speed

        if not entering_intersection and (distance from self.position to nearby_intersection) < TRIGGER_DISTANCE_TO_SLOWDOWN:
            entering_intersection = True
            select_maneuver = Uniform(*current_lane.maneuvers)

            # assumption: there always will be a maneuver
            if select_maneuver.connectingLane != None:
                current_centerline = concatenateCenterlines([current_centerline, select_maneuver.connectingLane.centerline, \
                    select_maneuver.endLane.centerline])
            else:
                current_centerline = concatenateCenterlines([current_centerline, select_maneuver.endLane.centerline])

            current_lane = select_maneuver.endLane
            end_lane = current_lane
            nearby_intersection = current_lane.maneuvers[0].intersection

            if select_maneuver.type != ManeuverType.STRAIGHT:
                in_turning_lane = True
                target_speed = TARGET_SPEED_FOR_TURNING

        if (end_lane is not None) and (self.position in end_lane):
            in_turning_lane = False
            entering_intersection = False 
            target_speed = original_target_speed
            
        nearest_line_points = current_centerline.nearestSegmentTo(self.position)
        nearest_line_segment = PolylineRegion(nearest_line_points)
        cte = nearest_line_segment.signedDistanceTo(self.position)

        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)
        past_steer_angle = current_steer_angle
        past_speed = current_speed

        take FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)

    
behavior FollowTrajectoryBehavior(target_speed = 10, trajectory = None):
    assert trajectory is not None
    assert isinstance(trajectory, list)

    trajectory_line = concatenateCenterlines(trajectory)

    # check whether self agent is vehicle:
    if self.blueprint:
        is_vehicle = self.blueprint in carModels
    else:
        # assume it is a car
        is_vehicle = True

    dt = simulation().timestep

    # instantiate longitudinal and latitudinal pid controllers
    if is_vehicle:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0, dt=dt)

    else:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)

    past_steer_angle = 0

    while True:
        if self.speed is not None:
            current_speed = self.speed
        else:
            current_speed = 0

        cte = trajectory_line.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)
        past_steer_angle = current_steer_angle


