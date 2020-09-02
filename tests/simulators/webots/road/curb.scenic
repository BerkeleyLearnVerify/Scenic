
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'simple.wbt')

from scenic.simulators.webots.road.model import *

ego = Car

spot = OrientedPoint on visible curb
badAngle = Options([1.0, -1.0]) * Range(10, 20) deg
parkedCar = Car left of (spot offset by -0.5 @ 0),
				facing badAngle relative to roadDirection