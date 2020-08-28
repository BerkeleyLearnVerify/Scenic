"""Lead vehicle cuts in. Ego slows down and follows lead vehicle."""

param map = localPath('../OpenDrive/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.carla.model


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

initLaneSec = Uniform(*network.laneSections)
egoPt = OrientedPoint on initLaneSec.centerline

spot = OrientedPoint following roadDirection from egoPt by Range(10, 20)
spotLane = network.laneAt(spot)
parkingSpot = OrientedPoint on spotLane.rightEdge

leadCar = Car left of spot by 0.25,
	with regionContainedIn None

ego = Car behind leadCar by Range(20, 30),
	with behavior EgoBehavior(leadCar, slowDownDist=5.0),
	with regionContainedIn None,
	with speed Range(10, 20)
