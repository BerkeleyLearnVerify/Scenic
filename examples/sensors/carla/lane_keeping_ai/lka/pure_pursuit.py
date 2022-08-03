import numpy as np
from .get_target_point import get_target_point

# The above parameters will be used in the Carla simulation
# The simple simulation in tests/control/control.ipynb does not use these parameters


class PurePursuit:
    def __init__(self, k_dd, wheel_base=2.65, waypoint_shift=1.4):
        self.K_dd = k_dd
        self.wheel_base = wheel_base
        self.waypoint_shift = waypoint_shift

    def get_control(self, waypoints, speed):
        # transform x coordinates of waypoints such that coordinate origin is in rear wheel
        waypoints[:, 0] += self.waypoint_shift
        look_ahead_distance = np.clip(self.K_dd * speed, 3, 20)

        track_point = get_target_point(look_ahead_distance, waypoints)
        if track_point is None:
            return 0

        alpha = np.arctan2(track_point[1], track_point[0])

        # Change the steer output with the lateral controller.
        steer = np.arctan((2 * self.wheel_base * np.sin(alpha)) / look_ahead_distance)

        # undo transform to waypoints
        waypoints[:, 0] -= self.waypoint_shift
        return steer


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.int_term = 0
        self.derivative_term = 0
        self.last_error = None

    def get_control(self, measurement, dt):
        error = self.set_point - measurement
        self.int_term += error * self.Ki * dt
        if self.last_error is not None:
            self.derivative_term = (error - self.last_error) / dt * self.Kd
        self.last_error = error
        return self.Kp * error + self.int_term + self.derivative_term


class PurePursuitPlusPID:
    def __init__(self, controller_attrs=None):
        attrs = {"k_p": 2, "k_i": 0, "k_d": 0, "k_dd": 0.4}
        if controller_attrs is not None:
            attrs.update(controller_attrs)
        self.pure_pursuit = PurePursuit(attrs["k_dd"])
        self.pid = PIDController(attrs["k_p"], attrs["k_i"], attrs["k_d"], 0)

    def get_control(self, waypoints, speed, desired_speed, dt):
        self.pid.set_point = desired_speed
        a = self.pid.get_control(speed, dt)
        steer = self.pure_pursuit.get_control(waypoints, speed)
        a = np.clip(a, 0.0, 1.0)
        steer = np.clip(steer, -1.0, 1.0)
        return a, steer
