"""Lead vehicle cuts in. Ego slows down and follows lead vehicle."""

import scenic.simulators.carla.actions as actions
import random

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *


# ============================================================================
# -- BEHAVIORS ---------------------------------------------------------------
# ============================================================================

behavior LeadBehavior():
	take actions.SetThrottleAction(0.3)
	for _ in range(3):
		take None

	take actions.SetThrottleAction(0.7)

	while True:
		take None

behavior EgoBehavior():
	take actions.SetThrottleAction(0.3)
	while True:
		take None

laneSecs = []
for lane in network.lanes:
	for sec in lane.sections:
		laneSecs.append(sec)

# initLaneSec = Options(laneSecs)
# egoPt = Options(initLaneSec.centerline)

# Hard code for testing
initLaneSec = laneSecs[50]
egoPt = initLaneSec.centerline[5]

leadCar = Car following roadDirection from egoPt by (10, 20),
	with behavior LeadBehavior

ego = Car at egoPt,
	with behavior EgoBehavior