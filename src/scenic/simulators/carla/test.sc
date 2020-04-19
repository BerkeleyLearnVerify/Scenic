import scenic.simulators.carla.behaviors as behaviors
from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, './Town01.xodr')
from scenic.simulators.carla.road_model import *  # NOTE: Must be after setting map path


ego = Car with behavior behaviors.StillBehavior
