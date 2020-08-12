# Traffic Scenario 04: Obstacle avoidance with prior action
# Definition: While performing a maneuver, the ego-vehicle finds an obstacle / unexpected entity on the road and must perform an emergency brake or an avoidance maneuver. 
# ego at intersection where a maneuver is available, ego takes any turn. obstacle is stationary according to sheets... but i want it to move

from scenic.simulators.lgsvl.simulator import LGSVLSimulator
from scenic.simulators.lgsvl.map import setMapPath
setMapPath(__file__, 'maps/borregasave.xodr')
from scenic.simulators.lgsvl.model import *

simulator = LGSVLSimulator('BorregasAve')
param time_step = 1.0/10

# CONSTANTS
index1 = Uniform(0, 1, 2, 3)

# GEOMETRY
fourLane = []
for i in network.intersections:
	if (len(i.incomingLanes) >= 8):
		fourLane.append(i)

intersection = Uniform(*fourLane)
lane = intersection.incomingLanes[index1]
pos = (OrientedPoint at lane.centerline[-1]) offset by (-2, 2) @ 0 # at last stretch of centerline, off center by at most 2
turn = Uniform(*lane.maneuvers)
pt = Point on turn.connectingLane.centerline # point in the ego's path that the pedestrian should walk towards

# BEHAVIOR
behavior CrossingBehavior():
	randomSpeedup = (0, 1)
	startWalkingDist = 100#(10, 15)
	while True:
		egoDist = distance from ego to pt
		walkDist = distance from p to pt
		if (ego.speed == 0):
			egoSpeed = 1
		else:
			egoSpeed = ego.speed
		walkSpeed = randomSpeedup + ((egoSpeed * walkDist) / egoDist)
		if(egoDist <= startWalkingDist):
			take SetSpeedAction(30)
		else:
			take SetSpeedAction(0.0)

behavior EgoBehavior():
	throttleStrength = (0, 1)
	while True:
		gain = 0.1
		delta = self.heading relative to turn.connectingLane.centerline.orientation
		take SetSteerAction(-gain * delta), SetThrottleAction(throttleStrength)


# PLACEMENT
ego = EgoCar at pos, facing roadDirection, with speed 5, with behavior EgoBehavior

# change this - crossings added
# (using intersection boundary for LGSVL version since maps have no sidewalks)
walkerSpawn = Point on visible intersection.boundary
p = Pedestrian at walkerSpawn offset by (-2,2) @ (-2,2),
	facing toward pt,
	with regionContainedIn None,
	with behavior CrossingBehavior


