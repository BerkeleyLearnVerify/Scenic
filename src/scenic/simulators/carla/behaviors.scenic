
from scenic.domains.driving.behaviors import *	# use common driving behaviors

try:
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior WalkForwardBehavior():
	while True:
		take SetSpeedAction(0.5)
