from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car with visibleDistance 60
c2 = new Car visible
platoon = createPlatoonAt(c2, 6, dist=Range(2, 8))