"""Scenic script for testing purposes only."""

from scenic.simulators.domains.driving.network import loadLocalNetwork
loadLocalNetwork(__file__, '../OpenDrive/Town01.xodr')

from scenic.simulators.carla.model import *
import scenic.simulators.carla.behaviors as behaviors

simulator = CarlaSimulator('Town01')


ego = Car with behavior behaviors.ConstantThrottleBehavior(0.5)

otherCar = Car ahead of ego,
    with behavior behaviors.ConstantThrottleBehavior(0.7)

obstacle = Pedestrian with behavior behaviors.WalkForwardBehavior
