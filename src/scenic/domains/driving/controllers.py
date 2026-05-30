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

import numpy as np


class LongitudinalController(ABC):
    @abstractmethod
    def compute_throttle(self):
        pass


class LateralController(ABC):
    @abstractmethod
    def compute_steering(self):
        pass


class PIDController:
    def __init__(self, K_P=0.5, K_D=0.1, K_I=0.2, dt=0.1):
        self.kp = K_P
        self.ki = K_I
        self.kd = K_D
        self.dt = dt
        self.i_term = 0
        self.last_error = None
        self.windup_guard = 20.0

    def run_step(self, error):
        # Compute terms
        p_term = error
        self.i_term += np.clip(error * self.dt, -self.windup_guard, self.windup_guard)
        d_term = (error - self.last_error) / self.dt if self.last_error else 0

        # Remember last error for next calculation
        self.last_error = error

        output = (self.kp * p_term) + (self.ki * self.i_term) + (self.kd * d_term)
        return np.clip(output, -1, 1)


class PIDLongitudinalController(PIDController):
    """Longitudinal control using a PID to reach a target speed.

    Arguments:
        K_P: Proportional gain
        K_D: Derivative gain
        K_I: Integral gain
        dt: time step
    """


class PIDLateralController(PIDController):
    """Lateral control using a PID to track a trajectory.

    Arguments:
        K_P: Proportional gain
        K_D: Derivative gain
        K_I: Integral gain
        dt: time step
    """
