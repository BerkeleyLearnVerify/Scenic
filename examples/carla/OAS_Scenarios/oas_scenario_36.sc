
import scenic.simulators.carla.actions as actions
import time
from shapely.geometry import LineString
from scenic.core.regions import regionFromShapelyObject
from scenic.simulators.domains.driving.network import loadNetwork
from scenic.simulators.domains.driving.roads import ManeuverType
loadNetwork('/home/carla_challenge/Downloads/Town03.xodr')

from scenic.simulators.carla.model import *
simulator = CarlaSimulator('Town03')

MAX_BREAK_THRESHOLD = 1
TERMINATE_TIME = 20

twoLane_roads = []
for r in network.roads:
	if len(r.lanes) == 2:
		twoLane_roads.append(r)

selected_road = Uniform(*twoLane_roads)
lane = selected_road.lanes[1]

ego = Car on lane

spot = OrientedPoint on visible curb
perturbation_angle = (-10, 10) deg

parkedCar = Car at (left of spot),
				facing perturbation_angle relative to roadDirection

doubleParkedCar = Car at (left of parkedCar)