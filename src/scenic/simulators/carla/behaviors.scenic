
from scenic.simulators.carla.actions import *

behavior WalkForwardBehavior():
	while True:
		take SetSpeedAction(0.5)
