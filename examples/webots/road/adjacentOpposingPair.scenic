
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'southside2.wbt')

from scenic.simulators.webots.road.model import *

ego = Car with visibleDistance 20
c2 = Car visible
c3 = Car at c2 offset by Range(-10, 1) @ 0
require abs(relative heading of c3 from c2) >= 150 deg