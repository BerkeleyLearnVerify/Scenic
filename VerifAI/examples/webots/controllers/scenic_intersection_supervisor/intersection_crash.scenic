
import guideways_accident

from scenic.simulators.webots.guideways.model import *

# find various guideways
egoGuideway = intersection.mainGuideway
turnGuideway = None
blockingGuideway = None
for guideway in intersection.vehicleGuideways:
	if guideway.direction == 'left':
		turnGuideway = guideway
	elif guideway.origin_lane == '2':
		blockingGuideway = guideway
assert turnGuideway
assert blockingGuideway

param turnWaypoints = turnGuideway.median.points

# some Regions
belowIntersection = PolygonalRegion([[-25, 12], [-5, 12], [-5, 5], [-25, 5]])
aboveIntersection = PolygonalRegion([[-20, 29], [-20, 60], [-5, 60], [-5, 29]])
justAboveIntersection = PolygonalRegion([[-20, 29], [-20, 34], [-5, 34], [-5, 29]])

# Uber
ego = Car on egoGuideway.median.intersect(aboveIntersection),
		  with speed Range(30, 50),
		  with webotsType 'Ego'

# Turning car
turningCar = Car on turnGuideway.median.intersect(belowIntersection),
				 with delay Range(0, 3),
				 with webotsType 'TurningCar'

# A car blocking part of the Ego's view
laneNoise = Range(-0.5, 0.5)
spot = OrientedPoint on blockingGuideway.median.intersect(justAboveIntersection)
Car at spot offset by laneNoise @ 0

# Another car behind that
spot2 = OrientedPoint at spot offset by 0 @ Range(-5, -8)
Car at spot2 offset by laneNoise @ 0

