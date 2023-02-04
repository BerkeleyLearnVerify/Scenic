from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, 'OpenDrive/Town01.xodr')
from scenic.simulators.carla.model import *

ego = Car with visibleDistance 20
c2 = Car visible
c3 = Car at c2 offset by Range(-10, 1) @ 0
require abs(relative heading of c3 from c2) >= 150 deg