import scenic.simulators.carla.scripts.behaviors as behaviors

from scenic.simulators.carla.maps.map import setMapPath
setMapPath(__file__, '../maps/Town01.xodr')
from scenic.simulators.carla.models.model import *


ego = Car with behavior behaviors.AccelerateForwardBehavior
otherCar = Car with behavior behaviors.AccelerateForwardBehavior
#obstacle = Pedestrian with behavior behaviors.WalkForwardBehavior
