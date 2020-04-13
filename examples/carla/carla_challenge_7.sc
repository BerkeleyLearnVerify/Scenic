from scenic.simulators.carla.map import setMapPath
setMapPath(__file__, 'OpenDrive/Town01.xodr')
from scenic.simulators.carla.model import *

ego = Car with visibleDistance 10
c2 = Car on intersection

require abs(relative heading of c2 from ego) >= 80 deg
require abs(relative heading of c2 from ego) <= 110 deg

