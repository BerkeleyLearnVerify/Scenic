
from scenic.domains.driving.behaviors import *	# use common driving behaviors

try:
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior AutopilotBehavior():
	take SetAutopilotAction(True)

behavior WalkForwardBehavior(speed=0.5):
	take SetWalkingDirectionAction(self.heading)
	take SetWalkingSpeedAction(speed)

behavior WalkBehavior(maxSpeed=1.4):
	take SetWalkAction(True, maxSpeed)
