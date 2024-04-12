
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'road.wbt')

from scenic.simulators.webots.road.model import *

# Pick location for blockage
curb = Uniform(*workspace.slowCurbs)
blockageSite = new OrientedPoint on curb

# Place cones
spot1 = new OrientedPoint left of blockageSite by Range(0.3, 1)
cone1 = new TrafficCone at spot1, facing Range(0, 360) deg

spot2 = new OrientedPoint ahead of spot1 by Range(-1.5, -0.75) @ Range(1, 4)
cone2 = new TrafficCone at spot2, facing Range(0, 360) deg

cone3 = new TrafficCone ahead of spot2 by Range(-1.5, -0.75) @ Range(1, 4),
	facing Range(0, 360) deg,
	with color [0, 0, Range(0.5, 1)]

# Place disabled car ahead of cones
new SmallCar ahead of spot2 by Range(-1, 1) @ Range(4, 10), facing Range(0, 360) deg

# Place ego car some way back from the site
spot = blockageSite offset along roadDirection by Range(-3.5, -0.5) @ Range(-3, -20)
ego = new ToyotaPrius at spot, with roadDeviation Range(-10, 10) deg

# Place another car
new SmallCar on visible road, with roadDeviation Range(-10, 10) deg