import scenic.simulators.carla.scripts.behaviors as behaviors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.models.model import *


ego = Car with behavior behaviors.AccelerateForwardBehavior
otherCar = Car with behavior behaviors.AccelerateForwardBehavior
#obstacle = Pedestrian with behavior behaviors.WalkForwardBehavior
