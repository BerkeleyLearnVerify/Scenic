from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

numCars = 3

ego = Car with visibleDistance 60

spot = OrientedPoint on visible curb
parked = Car left of (spot offset by -0.25 @ 0)
platoon = createPlatoonAt(parked, numCars, dist=(2, 5))
last = platoon[-1]

require[0.5] abs(relative heading of last from parked) >= 60 deg