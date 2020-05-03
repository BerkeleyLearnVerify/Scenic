import scenic.simulators.carla.actions as actions


behavior AccelerateForwardBehavior():
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)

behavior AccelerateBackwardBehavior():
	take actions.SetReverseAction(True)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)

behavior WalkForwardBehavior():
	take actions.SetVelocityAction(0.5)
