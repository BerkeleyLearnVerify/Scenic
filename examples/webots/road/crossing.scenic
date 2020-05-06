
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'richmond.wbt')

from scenic.simulators.webots.road.model import *

ped = Pedestrian on crossing

spot = ped offset along roadDirection by (-5, 5) @ (-3, -20)

ego = ToyotaPrius at spot, with roadDeviation (-10, 10) deg
