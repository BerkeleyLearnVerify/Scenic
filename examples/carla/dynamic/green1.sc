"""Lead vehicle cuts in. Ego slows down and follows lead vehicle."""

from scenic.domains.driving.network import loadLocalNetwork
loadLocalNetwork(__file__, '../OpenDrive/Town01.xodr')
from scenic.simulators.carla.model import *


# ============================================================================
# -- BEHAVIORS ---------------------------------------------------------------
# ============================================================================

behavior LeadBehavior():
	for _ in range(3):
		take None

	take SetThrottleAction(0.3)
	while True:
		take None

behavior SuddenBrakeBehavior():
	"""Apply brakes and emergency brake suddenly."""
	while self.speed > 5:
		take SetBrakeAction(0.5)
	# take actions.SetHandBrakeAction(True)
	take SetBrakeAction(0.0)  # Release foot brake after slowing down


behavior EgoBehavior(leadCar, slowDownDist=5.0):
	"""Drive forward, slowing down if needed."""
	take SetThrottleAction(0.6)
	while True:
		if (distance from self to leadCar) <= slowDownDist:
			SuddenBrakeBehavior()
		else:
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

spot = OrientedPoint following roadDirection from egoPt by (10, 20)
spotLane = network.laneAt(spot)
parkingSpot = OrientedPoint on spotLane.rightEdge

leadCar = Car left of spot by 0.25,
	with regionContainedIn None

ego = Car behind leadCar by (20, 30),
	with behavior EgoBehavior(leadCar, slowDownDist=5.0),
	with regionContainedIn None,
	with speed (10, 20)
