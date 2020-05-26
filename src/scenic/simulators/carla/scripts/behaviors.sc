import scenic.simulators.carla.actions as actions


behavior AccelerateForwardBehavior():
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)

behavior LanekeepingBehavior():
	take actions.SetReverseAction(False)
	take actions.SetHandBrakeAction(False)
	take actions.SetThrottleAction(0.5)
	while True:
		delta = 1
		#delta = self.heading - roadDirection
		take actions.SetSteerAction(delta)

behavior WalkForwardBehavior():
	take actions.SetSpeedAction(0.5)

behavior A(x):
	take actions.SetThrottleAction(x)

behavior B():
	take actions.SetThrottleAction(0.7)
