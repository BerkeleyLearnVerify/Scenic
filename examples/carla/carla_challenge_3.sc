from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, 'OpenDrive/Town01.xodr')
from scenic.simulators.carla.road_model import *

ego = Car 
c2 = Trash at ego offset by 0 @ (15, 20)