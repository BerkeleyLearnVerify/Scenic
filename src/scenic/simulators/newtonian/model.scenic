"""Scenic world model for traffic scenarios in CPU-based Newtonian simulator.

The model currently supports vehicles and map parsing by providing the 'map' parameter
as a path to the OpenDrive map file to be used.
"""

from scenic.domains.driving.model import *

from scenic.simulators.newtonian.behaviors import *
from scenic.simulators.utils.colors import Color

from scenic.simulators.newtonian.simulator import NewtonianSimulator    # for use in scenarios
from scenic.simulators.newtonian.actions import *

if 'map' not in globalParameters:
    raise RuntimeError('need to specify map before importing Newtonian simulator model '
                       '(set the global parameter "map")')
if 'render' not in globalParameters:
    render = True
else:
    render = globalParameters.render
simulator NewtonianSimulator(globalParameters.map, render=render)

class NewtonianActor(DrivingObject):

    position: None
    velocity: None
    throttle: 0
    steer: 0
    brake: 0
    hand_brake: 0
    reverse: 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    def setPosition(self, pos, elevation):
        self.position = pos

    def setVelocity(self, vel):
        self.velocity = vel

    def setThrottle(self, throttle):
        self.throttle = throttle

    def setSteering(self, steering):
        self.steer = steering

    def setBraking(self, braking):
        self.brake = braking

    def setHandbrake(self, handbrake):
        self.hand_brake = handbrake

    def setReverse(self, reverse):
        self.reverse = reverse

class Vehicle(Vehicle, NewtonianActor):

    pass

class Car(Vehicle, Steers):
    pass

class Debris:
	"""Abstract class for debris scattered randomly in the workspace."""
	position: Point in workspace
	heading: Range(0, 360) deg