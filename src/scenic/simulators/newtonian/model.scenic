"""Scenic world model for traffic scenarios in the Newtonian simulator.

This model implements the basic :obj:`~scenic.domains.driving.model.Car` class from the
:obj:`scenic.domains.driving` domain.
Vehicles support the basic actions and behaviors from the driving domain.

A path to a map file for the scenario should be provided as the ``map`` global parameter;
see the driving domain's documentation for details.
"""

from scenic.domains.driving.model import *  # includes basic actions and behaviors

from scenic.simulators.utils.colors import Color

from scenic.simulators.newtonian.simulator import NewtonianSimulator    # for use in scenarios

if 'render' not in globalParameters:
    render = True
else:
    render = globalParameters.render
simulator NewtonianSimulator(network, render=render)

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
