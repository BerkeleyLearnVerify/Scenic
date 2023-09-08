from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car

spot = new OrientedPoint on visible curb
badAngle = Uniform(1.0, -1.0) * Range(10, 20) deg
parkedCar = new Car left of (spot offset by -0.5 @ 0),
                facing badAngle relative to roadDirection
