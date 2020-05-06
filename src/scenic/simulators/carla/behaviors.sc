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
		delta = self.heading - self.roadDirection
		print(delta)
		take actions.SetSteerAction(0.1)

behavior WalkForwardBehavior():
	take actions.SetVelocityAction(0.5)
