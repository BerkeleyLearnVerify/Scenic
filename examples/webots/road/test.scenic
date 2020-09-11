
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'southside2.wbt')

from scenic.simulators.webots.road.model import *

ego = Car with visibleDistance 20
c2 = Car visible, with roadDeviation Range(10, 20) deg
require abs((relative heading of c2) - 90 deg) <= 10 deg
