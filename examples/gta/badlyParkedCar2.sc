from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.gta_model import *

ego = Car

spot = OrientedPoint on visible curb
badAngle = Options([1.0, -1.0]) * (10, 20) deg
parkedCar = Car left of (spot offset by -0.5 @ 0), \
				facing badAngle relative to roadDirection
