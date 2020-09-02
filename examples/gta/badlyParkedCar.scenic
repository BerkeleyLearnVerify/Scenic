from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

spot = OrientedPoint on curb
badAngle = Options([1.0, -1.0]) * Range(10, 20) deg
parkedCar = Car left of (spot offset by -0.5 @ 0), facing badAngle relative to roadDirection

ego = Car at parkedCar offset by Range(-20, 0) @ Range(-30, 30)