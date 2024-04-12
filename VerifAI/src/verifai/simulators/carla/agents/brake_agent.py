import carla
import sys

from agents.navigation.agent import *
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import distance_vehicle, draw_waypoints

import numpy as np

'''This agent doesn't move.'''
class BrakeAgent(Agent):

    def __init__(self, vehicle, opt_dict=None):
        super(BrakeAgent, self).__init__(vehicle)

    def run_step(self):
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = True
        return control
