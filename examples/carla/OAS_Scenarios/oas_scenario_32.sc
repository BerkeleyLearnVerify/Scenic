from scenic.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

param map = localPath('../OpenDrive/Town10HD.xodr')
param carla_map = 'Town10HD'

from scenic.domains.driving.behaviors import *

model scenic.domains.driving.model

fourLane = []
for i in network.intersections:
	if (len(i.incomingLanes) >= 8):
		fourLane.append(i)

intersection = fourLane[0] # hard coded so I don't have to fix the double-sampling
maneuvers = intersection.maneuvers

straight_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.STRAIGHT:
		straight_manuevers.append(m)

straight_maneuver = straight_manuevers[0]
startLane = straight_maneuver.startLane
connectingLane = straight_maneuver.connectingLane
endLane = straight_maneuver.endLane

centerlines = [startLane.centerline, connectingLane.centerline, endLane.centerline]


leftTurn_manuevers = []
for m in maneuvers:
	if m.type == ManeuverType.LEFT_TURN:
		leftTurn_manuevers.append(m)

leftTurn_maneuver = leftTurn_manuevers[1]
ego_L_startLane = leftTurn_maneuver.startLane
ego_L_connectingLane = leftTurn_maneuver.connectingLane
ego_L_endLane = leftTurn_maneuver.endLane

ego_L_centerlines = [ego_L_startLane.centerline, ego_L_connectingLane.centerline, ego_L_endLane.centerline]


# PLACEMENT
ego = Car on ego_L_startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=10, trajectory=ego_L_centerlines)

other = Car on startLane.centerline,
		with blueprint 'vehicle.tesla.model3',
		with behavior FollowTrajectoryBehavior(target_speed=15, trajectory=centerlines)