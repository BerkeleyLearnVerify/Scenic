from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, 'OpenDrive/Town01.xodr')
from scenic.simulators.carla.model import *

ego = Car
