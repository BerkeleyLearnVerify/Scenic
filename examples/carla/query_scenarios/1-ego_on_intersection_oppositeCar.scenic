param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town10HD.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

intersection = Uniform(*network.intersections)
incomingLanes = filter(lambda i: i.sections[0]._laneToRight is None, intersection.incomingLanes)
incomingLane = Uniform(*incomingLanes)
right_maneuvers = filter(lambda i: i.type == ManeuverType.RIGHT_TURN, incomingLane.maneuvers)
right_maneuver = Uniform(*right_maneuvers)

ego = Car on right_maneuver.connectingLane,
		facing Range(-15,15) deg relative to roadDirection,
		with visibleDistance 15,
		with viewAngle 135 deg

selected_road = right_maneuver.endLane.road
otherCar = Car on visible selected_road,
	facing Range(-15,15) deg relative to roadDirection

require abs(relative heading of ego from otherCar) > 150 deg