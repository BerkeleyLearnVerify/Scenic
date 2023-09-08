from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car

for i in range(4):
    spot = new OrientedPoint on visible curb
    new Car left of (spot offset by -0.25 @ 0)