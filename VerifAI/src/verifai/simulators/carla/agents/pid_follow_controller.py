import carla
from agents.navigation.agent import *
from agents.navigation.controller import VehiclePIDController

import numpy as np
from collections import deque

from pid_agent import *

class PIDLocationController():
    def __init__(self, vehicle, clear_dist, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        self._vehicle = vehicle
        self.clear_dist = clear_dist
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._dt = dt
        self._e_buffer = deque(maxlen=30)


    def run_step(self, location):
        '''Run step with target LOCATION.'''
        return self._pid_control(location, self._vehicle.get_transform())


    def _pid_control(self, target_location, cur_transform):
        _e = target_location.distance(cur_transform.location) - self.clear_dist
        self._e_buffer.append(_e)

        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return (self._K_P * _e) + (self._K_D * _de / self._dt) + (self._K_I * _ie * self._dt)


class PIDFollowController(VehiclePIDController):
    '''Adds PID control for longitudinal position.'''
    def __init__(self, vehicle, clear_dist,
                 args_lateral={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0},
                 args_longitudinal={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0},
                 args_location={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0}):
        super().__init__(vehicle, args_lateral=args_lateral,
                     args_longitudinal=args_longitudinal)
        self.loc_controller = PIDLocationController(
            vehicle, clear_dist, **args_location)


    def run_step(self, target_speed, waypoint, location=None):
        """
        Defaults to maintaining TARGET_SPEED toward WAYPOINT. If LOCATION,
        instead control throttle to minimize distance to LOCATION
        """
        control = super().run_step(target_speed, waypoint)
        if location:
            brake_throttle = self.loc_controller.run_step(location)
            if brake_throttle < 0:
                control.brake = np.clip(-brake_throttle, 0.0, 1.0)
                control.throttle = 0.0
            else:
                control.throttle = np.clip(brake_throttle, 0.0, 1.0)
                control.brake = 0.0

        return control
        
