"""Lead vehicle cuts in. Ego slows down and follows lead vehicle."""

from scenic.domains.driving.network import loadLocalNetwork
loadLocalNetwork(__file__, '../OpenDrive/Town01.xodr')

from scenic.simulators.carla.model import *

simulator = CarlaSimulator('Town01')


# ============================================================================
# -- BEHAVIORS ---------------------------------------------------------------
# ============================================================================

behavior LeadBehavior():
	take SetThrottleAction(0.3)
	for _ in range(3):
		wait

	take SetThrottleAction(0.7)

	while True:
		wait

behavior EgoBehavior():
	take SetThrottleAction(0.3)

laneSecs = []
for lane in network.lanes:
	for sec in lane.sections:
		laneSecs.append(sec)

initLaneSec = Options(laneSecs)

ego = Car on initLaneSec.centerline,
	with behavior EgoBehavior

leadCar = Car following roadDirection by (10, 20),
	with behavior LeadBehavior