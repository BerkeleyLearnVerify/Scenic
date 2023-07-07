from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, localPath('../../../../examples/gta/map.npz'))

from scenic.simulators.gta.model import *

ego = new Car
c2 = new Car visible
c3 = new Car visible
require (relative heading of c3 from c2) >= 150 deg
require (distance from c2 to c3) <= 10