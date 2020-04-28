import scenic.simulators.carla.actions as actions


behavior AccelerateForwardBehavior():
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)
	while True:
		take None
