from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car
c2 = new Car visible
c3 = new Car at c2 offset by Range(-10, 1) @ 0
require (relative heading of c3 from c2) >= 150 deg