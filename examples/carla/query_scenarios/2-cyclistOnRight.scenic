param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.simulators.carla.model

intersection = Uniform(*network.intersections)
incomingLanes = filter(lambda i: i.sections[0]._laneToRight is None, intersection.incomingLanes)
incomingLane = Uniform(*incomingLanes)
straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, incomingLane.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
potential_lanes = [straight_maneuver.startLane, straight_maneuver.connectingLane, straight_maneuver.endLane]
selected_lane = Uniform(*potential_lanes)

ego = Car on selected_lane,
		facing Range(-15,15) deg relative to roadDirection,
		with visibleDistance 20,
		with viewAngle 135 deg

Bicycle on visible selected_lane,
	facing Range(-10, 10) deg relative to roadDirection