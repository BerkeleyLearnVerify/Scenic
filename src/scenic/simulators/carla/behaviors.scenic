
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

behavior CrossingBehavior(reference_actor, min_speed=1, threshold=10, final_speed=None):

    if not final_speed:
        final_speed = min_speed

    while (distance from self to reference_actor) > threshold:
        wait

    while True:
        distance_vec = self.position - reference_actor.position
        rotated_vec = distance_vec.rotatedBy(-reference_actor.heading)

        ref_dist = rotated_vec.y
        if ref_dist < 0:
            # The reference_actor has passed the crossing object, no need to keep monitoring the speed
            break

        actor_dist = rotated_vec.x

        ref_speed = reference_actor.speed
        ref_time = ref_speed / ref_dist

        actor_speed = actor_dist * ref_time
        if actor_speed < min_speed:
            actor_speed = min_speed

        if isinstance(self, _model.Walks):
            do WalkForwardBehavior(actor_speed)
        elif isinstance(self, _model.Steers):
            take SetSpeedAction(actor_speed)

    if isinstance(self, _model.Walks):
        do WalkForwardBehavior(final_speed)
    elif isinstance(self, _model.Steers):
        take SetSpeedAction(final_speed)
