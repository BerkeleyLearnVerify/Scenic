
from simulators.domains.driving.actions import *

behavior ConstantThrottleBehavior(x):
    while True:
        take SetThrottleAction(x), SetReverseAction(False), SetHandBrakeAction(False)

behavior FollowLaneBehavior(target_speed = 25, network = None):

    # instantiate longitudinal and latitudinal pid controllers
    dt = simulation().timestep
    _lon_controller = controllers.PIDLongitudinalController(dt=dt)
    _lat_controller = controllers.PIDLateralController(dt=dt)
    past_steer_angle = 0

    while True:
        cte = self.lane.centerline.signedDistanceTo(self.position)
        speed_error = target_speed - self.speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take FollowLaneAction(throttle=throttle,
                              current_steer=current_steer_angle,
                              past_steer=past_steer_angle)
        past_steer_angle = current_steer_angle


behavior FollowTrajectoryBehavior(target_speed = 25, trajectory = None):
    assert trajectory is not None

    trajectory_line = PolylineRegion.unionAll(trajectory)

    # instantiate longitudinal and latitudinal pid controllers
    dt = simulation().timestep
    _lon_controller = actions.PIDLongitudinalController(dt=dt)
    _lat_controller = actions.PIDLateralController(dt=dt)
    past_steer_angle = 0

    while True:
        cte = trajectory_line.signedDistanceTo(self.position)
        speed_error = target_speed - self.speed

        # compute throttle : Longitudinal Control
        throttle = _lon_controller.run_step(speed_error)

        # compute steering : Latitudinal Control
        current_steer_angle = _lat_controller.run_step(cte)

        take FollowLaneAction(throttle=throttle,
                              current_steer=current_steer_angle,
                              past_steer=past_steer_angle)
        past_steer_angle = current_steer_angle
