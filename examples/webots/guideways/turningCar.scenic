
import tempe_crash

from scenic.simulators.webots.guideways.model import *

# find left-turn guideway
turnGuideway = None
for guideway in intersection.vehicleGuideways:
	if guideway.direction == 'left':
		turnGuideway = guideway
		break
assert turnGuideway

ego = Car in intersection.mainGuideway.region

turningCar = Car in turnGuideway.region, with requireVisible True
