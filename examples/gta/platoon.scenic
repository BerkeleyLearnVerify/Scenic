from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = Car with visibleDistance 60
c2 = Car visible
platoon = createPlatoonAt(c2, 6, dist=(2, 8))