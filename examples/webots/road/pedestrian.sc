
from scenic.simulators.webots.road.world import setLocalWorld
setLocalWorld(__file__, 'mcity.wbt')

from scenic.simulators.webots.road.road_model import *

ego = ToyotaPrius with visibleDistance 30

Pedestrian on visible sidewalk
