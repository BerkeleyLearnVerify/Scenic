
import scenic.simulators.carla.actions as actions
from scenic.simulators.carla.model import roadDirection

behavior AccelerateForwardBehavior():
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)

behavior LanekeepingBehavior(gain=0.1):
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)
	while True:
		delta = self.heading relative to roadDirection
		take actions.SetSteerAction(-gain * delta)

behavior WalkForwardBehavior():
	take actions.SetSpeedAction(0.5)

behavior ConstantThrottleBehavior(x):
    take actions.SetThrottleAction(x)
