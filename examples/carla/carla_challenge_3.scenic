# Traffic Scenario 03: Obstacle avoidance without prior action
# Definition: The ego-vehicle encounters an obstacle / unexpected entity on the road and must perform an emergency brake or an avoidance maneuver.

import scenic.simulators.carla.actions as actions
from scenic.core.distributions import *
from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town03.xodr')
from scenic.simulators.carla.model import *
simulator = CarlaSimulator('Town01')

# CONSTANTS
dist = TruncatedNormal(0, 0.1, -0.25, 0.25)

# GEOMETRY
lane = Uniform(*network.lanes)
ideal = OrientedPoint on lane.centerline

# BEHAVIOR
behavior CrossingBehavior():
	randomSpeedup = Range(0, 1)
	startWalkingDist = Range(10, 15)
	
	while True:
		egoDist = distance from ego to pt
		walkDist = distance from p to pt
		if (ego.speed == 0):
			egoSpeed = 1
		else:
			egoSpeed = ego.speed
		walkSpeed = randomSpeedup + ((egoSpeed * walkDist) / egoDist)
		if(egoDist <= startWalkingDist):
			take actions.SetSpeedAction(walkSpeed)
		else:
			take actions.SetSpeedAction(0.0)

# PLACEMENT
ego = Car at ideal offset by resample(dist) @ 0,
	with speed 0

pt = Point following roadDirection from ego by Range(10, 30) # point in the ego's path that the pedestrian should walk towards

p = Pedestrian on visible sidewalk,
	facing toward pt, 
	with behavior CrossingBehavior

require (distance to intersection) > 20 
require 20 <= (distance from p to ego) <= 40



# NOTES
	# issue: sometimes the point isn't on the road?? (maybe fixed)