
from scenic.domains.driving.behaviors import *	# use common driving behaviors

try:
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior AutopilotBehavior():
	self.carlaActor.set_autopilot(True, simulation().tm.get_port())
	while True:
		take SetSpeedAction(0.5)

behavior WalkForwardBehavior(speed=0.5):
	take SetWalkingDirectionAction(0)
	take SetWalkingSpeedAction(speed)

behavior WalkBehavior(maxSpeed=1.4):
	self.carlaController.start()
	self.carlaController.go_to_location(simulation().world.get_random_location_from_navigation())
	self.carlaController.set_max_speed(maxSpeed)
	while True:
		wait
