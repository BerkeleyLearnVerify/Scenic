import scenic.simulators.carla.simulator as simulator
import scenic.simulators.carla.actions as actions

behavior TeleportForward():
''' Continually teleports actor forward in direction of it heading '''
	while True:
		take actions.OffsetAction(0.5)
