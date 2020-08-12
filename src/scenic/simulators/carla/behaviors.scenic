
from scenic.domains.driving.behaviors import *	# use common driving behaviors
from scenic.simulators.carla.actions import *

behavior WalkForwardBehavior():
	while True:
		take SetSpeedAction(0.5)
