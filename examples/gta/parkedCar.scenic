from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

spot = new OrientedPoint on curb
parkedCar = new Car left of (spot offset by -0.25 @ 0)

ego = new Car at parkedCar offset by Range(-20, 0) @ Range(-30, 30)