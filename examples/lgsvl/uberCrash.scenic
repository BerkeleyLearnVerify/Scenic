from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/borregasave.xodr')
from scenic.simulators.lgsvl.model import *
from scenic.simulators.lgsvl.behaviors import *
import matplotlib.pyplot as plt
simulator LGSVLSimulator('BorregasAve')
param time_step = 1.0/10

# CONSTANTS
uberSpeed = Range(30, 50)
turnSpeed = Range(20, 25)
brakeIntensity = Range(0.6, 0.8)

# LANES 
fourLane = filter(lambda i: i.is4Way, network.intersections)
intersection = Uniform(*fourLane)

# uber's lane
rightManeuvers = filter(lambda m: m.type == ManeuverType.RIGHT_TURN, intersection.maneuvers) # because only the rightmost lane will have a right maneuver
rightTurn = Uniform(*rightManeuvers)
egoStartLane = rightTurn.startLane

straightManeuvers = filter(lambda  m: m.type == ManeuverType.STRAIGHT, egoStartLane.maneuvers)  # problem with borregas - right lane is only for right turns
egoManeuver = Uniform(*rightManeuvers) # Uniform(*straightManeuvers)

# actor's lane
conflictingLefts = filter(lambda  m: m.type == ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers) 
actorTurn = Uniform(*conflictingLefts)
actorStart = actorTurn.startLane

# stopped car's lane
stationaryLane = egoStartLane.sections[-1].laneToLeft

# POSITIONS
pos1 = (OrientedPoint at stationaryLane.centerline[-1]) offset by Range(-2, 2) @ 0 
pos2 = (OrientedPoint at stationaryLane.centerline[-1]) offset by Range(-2, 2) @ 0
dist1 = Range(1, 5)
dist2 = Range(10, 15)

egoPos = (OrientedPoint at egoStartLane.centerline[-1]) offset by Range(-2, 2) @ 0 
egoDist = Range(20, 25)

actorPos = (OrientedPoint at actorStart.centerline[-1]) offset by Range(-2, 2) @ 0 
actorDist = Range(2, 5)

# trajectories
egoTrajectory = [egoManeuver.startLane.centerline, egoManeuver.connectingLane.centerline, egoManeuver.endLane.centerline]
actorTrajectory = [actorTurn.startLane.centerline, actorTurn.connectingLane.centerline, actorTurn.endLane.centerline]


# BEHAVIORS
behavior actorCarBehavior(target_speed, trajectory, brake):
	try: 
		do FollowTrajectoryBehavior(target_speed = turnSpeed, trajectory = actorTrajectory)

	interrupt when(actor in actorTurn.endLane):
		take SetThrottleAction(0.0), SetBrakeAction(brakeIntensity)

# PLACEMENT
# Cars blocking view
stopped1 = NPCCar following roadDirection from pos1 by -dist1
stopped2 = NPCCar following roadDirection from pos2 by -dist2

# Uber
ego = Car following roadDirection from egoPos by -egoDist,
	with speed uberSpeed,
	with behavior FollowTrajectoryBehavior(target_speed=uberSpeed, trajectory=egoTrajectory)

#assert (ego is right of stopped1)

# Turning Car
actor = Car following roadDirection from actorPos by -actorDist,
	with behavior actorCarBehavior(target_speed = turnSpeed, trajectory = actorTrajectory, brake = brakeIntensity),
	with speed turnSpeed
