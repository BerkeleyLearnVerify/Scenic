from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

spot = OrientedPoint on curb
parkedCar = Car left of (spot offset by -0.25 @ 0)

ego = Car at parkedCar offset by (-20, 0) @ (-30, 30)