import scenic.simulators.carla.behaviors as behaviors
from scenic.simulators.carla.road_model import *
from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, './Town01.xodr')


ego = Car with behavior behaviors.TeleportForwardBehavior
