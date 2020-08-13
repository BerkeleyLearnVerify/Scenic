"""Lead vehicle cuts in. Ego slows down and follows lead vehicle."""

param map = localPath('../OpenDrive/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.carla.model


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