import scenic.simulators.carla.behaviors as behaviors
from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, './Town01.xodr')
from scenic.simulators.carla.carla_model import *  # NOTE: Must be after setting map path


ego = Car with behavior behaviors.LanekeepingBehavior
#otherCar = Car with behavior behaviors.AccelerateBackwardBehavior
#obstacle = Pedestrian with behavior behaviors.WalkForwardBehavior
