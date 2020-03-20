from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = Car
c2 = Car visible
c3 = Car visible
require (relative heading of c3 from c2) >= 150 deg
require (distance from c2 to c3) <= 10