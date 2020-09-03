
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

def distance(pos1, pos2):
    """ pos1, pos2 = (x,y) """
    return math.sqrt(math.pow(pos1[0]-pos2[0],2) + math.pow(pos1[1]-pos2[1],2))

def distanceToAnyObjs(vehicle, thresholdDistance):
    """ checks whether there exists any obj
    (1) in front of the vehicle, (2) within thresholdDistance """
    objects = simulation().objects
    for obj in objects:
        if not (vehicle can see obj):
            continue
        if distance(vehicle.position, obj.position) < 0.1:
            # this means obj==vehicle
            pass
        elif distance(vehicle.position, obj.position) < thresholdDistance:
            return True
    return False

def distanceToObjsInLane(vehicle, thresholdDistance):
    """ checks whether there exists any obj
    (1) in front of the vehicle, (2) on the same lane, (3) within thresholdDistance """
    objects = simulation().objects
    network = _model.network
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

def setLaneFollowingPIDControllers(is_vehicle, dt):
    if is_vehicle: # Switch to LaneFollowing PID Controller
        lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0, dt=dt)

    else:
        lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)

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
    take SetReverseAction(False)
    take SetHandBrakeAction(False)
    take SetThrottleAction(0.5)

behavior WalkForwardBehavior():
    current_sidewalk = _model.network.sidewalkAt(self.position)
    end_point = Uniform(*current_sidewalk.centerline.points)
    end_vec = end_point[0] @ end_point[1]
    normal_vec = Vector.normalized(end_vec)
    take WalkTowardsAction(goal_position=normal_vec)
    take SetSpeedAction(speed=1)

behavior ConstantThrottleBehavior(x):
    take SetThrottleAction(x)

behavior FollowLaneBehavior(target_speed = 10, laneToFollow=None, is_oppositeTraffic=False):
    print("FOLLOW LANE BEHAVIOR")
    print("position: ", self.position)
    ## Follow's the lane on which the vehicle is at 
    ## As the vehicle reaches an intersection, any route (eg. straigth or turn maneuver) is randomly selected and followed
    
    past_steer_angle = 0
    past_speed = 0 # making an assumption here that the agent starts from zero speed
    if laneToFollow is None:
        current_lane = self.lane
    else:
        current_lane = laneToFollow

    # print("Beginning: All possible maneuvers: ", c.maneuvers)
    current_centerline = current_lane.centerline
    in_turning_lane = False # assumption that the agent is not instantiated within a connecting lane
    intersection_passed = False
    entering_intersection = False # assumption that the agent is not instantiated within an intersection
    end_lane = None
    original_target_speed = target_speed
    TARGET_SPEED_FOR_TURNING = 5 # KM/H
    TRIGGER_DISTANCE_TO_SLOWDOWN = 10 # FOR TURNING AT INTERSECTIONS
    nearby_intersection = current_lane.maneuvers[0].intersection

    # check whether self agent is vehicle:
    if hasattr(self, 'blueprint'):
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
            print("nearby intersection")
            entering_intersection = True
            intersection_passed = False
            # print("All possible maneuvers: ", current_lane.maneuvers)
            straight_manuevers = filter(lambda i: i.type == ManeuverType.STRAIGHT, current_lane.maneuvers)

            if len(straight_manuevers) > 0:
                select_maneuver = Uniform(*straight_manuevers)
            else:
                print("there exists no straight maneuver in the upcoming intersection")
                if len(current_lane.maneuvers) > 0:
                    select_maneuver = Uniform(*current_lane.maneuvers)
                else:
                    take SetBrakeAction()
                    break

            # assumption: there always will be a maneuver
            if select_maneuver.connectingLane != None:
                current_centerline = concatenateCenterlines([current_centerline, select_maneuver.connectingLane.centerline, \
                    select_maneuver.endLane.centerline])
                print("current_centerline updated")
            else:
                current_centerline = concatenateCenterlines([current_centerline, select_maneuver.endLane.centerline])
                # print("current_centerline updated without connecting lane")
                # if is_vehicle: # Switch to PID controller for turning
                #     _lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
                #     _lat_controller = controllers.PIDLateralController(K_P=0.3, K_D=0.2, K_I=0, dt=dt)

                # else:
                #     _lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
                #     _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)

            current_lane = select_maneuver.endLane
            end_lane = current_lane
            nearby_intersection = current_lane.maneuvers[0].intersection

            if select_maneuver.type != ManeuverType.STRAIGHT:
                in_turning_lane = True
                target_speed = TARGET_SPEED_FOR_TURNING

                print("VEHICLE ABOUT TO TURN & SWITCH TO TURNING PID")
                # if is_vehicle: # Switch to PID controller for turning
                #     _lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
                #     _lat_controller = controllers.PIDLateralController(K_P=0.3, K_D=0.2, K_I=0, dt=dt)

                # else:
                #     _lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
                #     _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
                do TurnBehavior(trajectory = current_centerline)


        if (end_lane is not None) and (self.position in end_lane) and not intersection_passed:
            intersection_passed = True
            in_turning_lane = False
            entering_intersection = False 
            target_speed = original_target_speed
            print("VEHICLE IS ENTERING ENDLANE & SWITCH TO LANEFOLLOWING PID")
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
        past_steer_angle = current_steer_angle
        past_speed = current_speed

        take FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)

    
behavior FollowTrajectoryBehavior(target_speed = 10, trajectory = None):
    assert trajectory is not None
    assert isinstance(trajectory, list)

    print("FollowTrajectoryBehavior")
    print("position: ", self.position)

    brakeIntensity = 1.0
    distanceToEndpoint = 5 # meters

    traj_centerline = [traj.centerline for traj in trajectory]
    trajectory_centerline = concatenateCenterlines(traj_centerline)
    # print("trajectory_centerline: ", trajectory_centerline)

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
    end_intersection = trajectory[-1].maneuvers[0].intersection

    while True:
        if self in _model.network.intersectionRegion:
            print("IN INTERSECTION: TURNING")
            do TurnBehavior(trajectory_centerline)

        if (distance from self to end_intersection) < distanceToEndpoint:
            print("distance to endpoint reached")
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

        take FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)
        past_steer_angle = current_steer_angle
    print("FollowTrajectoryBehavior had finished")

behavior TurnBehavior(trajectory, target_speed=6):
    print("TurnBehavior Executes")

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

    if is_vehicle:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.5, K_D=0.1, K_I=0, dt=dt)

    else:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)

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

        take FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)
        past_steer_angle = current_steer_angle
    
    print("Turn Behavior ended")


behavior LaneChangeBehavior(laneSectionToSwitch, is_oppositeTraffic=False, target_speed=10):
    print("LANE CHANGE BEHAVIOR")
    brakeIntensity = 1.0
    distanceToEndpoint = 3 # meters

    current_lane = laneSectionToSwitch.lane
    traj_centerline = [current_lane.centerline]
    trajectory_centerline = concatenateCenterlines(traj_centerline)
    nearby_intersection = current_lane.maneuvers[0].intersection

    # check whether self agent is vehicle:
    if hasattr(self, 'blueprint'):
        if (self.blueprint in carModels) or (self.blueprint in truckModels):
            is_vehicle = True
        else:
            is_vehicle = False
    else:
        # assume it is a car
        is_vehicle = True

    dt = simulation().timestep

    # instantiate longitudinal and latitudinal pid controllers
    if is_vehicle:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.08, K_D=0.1, K_I=0, dt=dt)

    else:
        _lon_controller = controllers.PIDLongitudinalController(K_P=0.25, K_D=0.025, K_I=0.0, dt=dt)
        _lat_controller = controllers.PIDLateralController(K_P=0.1, K_D=0.1, K_I=0.0, dt=dt)

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
                    take SetBrakeAction()
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

        take FollowLaneAction(throttle=throttle, current_steer=current_steer_angle, past_steer=past_steer_angle)
        past_steer_angle = current_steer_angle

    print("breaking off of LaneChangeBehavior")


