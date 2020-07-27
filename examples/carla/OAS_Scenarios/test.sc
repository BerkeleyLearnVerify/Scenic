
import scenic.simulators.carla.actions as actions
import time

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *
from scenic.simulators.carla.behaviors import *

simulator = CarlaSimulator('Town01')

# roads = network.roads
# select_road = Uniform(*roads)
# possible_lanes = select_road.lanes 
# select_lane = Uniform(*possible_lanes)

threeWayIntersections = []
for intersection in network.intersections:
	if intersection.is3Way:
		threeWayIntersections.append(intersection)

# intersection = Uniform(*fourWayIntersections)
intersection = threeWayIntersections[5]
maneuvers = intersection.maneuvers

leftTurn_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.LEFT_TURN:
		leftTurn_manuevers.append(m)

leftTurn_maneuver = leftTurn_manuevers[1]
L_startLane = leftTurn_maneuver.startLane
L_connectingLane = leftTurn_maneuver.connectingLane
L_endLane = leftTurn_maneuver.endLane

L_centerlines = [L_startLane.centerline, L_connectingLane.centerline, L_endLane.centerline]

ego = Car at L_startLane[-1],
		with behavior FollowTrajectoryBehavior(15, trajectory=L_centerlines),
		with blueprint 'vehicle.tesla.model3'