import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.model import *

"""
Leading vehicle decelerates suddently due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 02.
"""

#CONSTANTS
BRAKE_ACTION = 1.0
THROTTLE_ACTION = 0.6
EGO_TO_LEADCAR = -30
EGO_BRAKING_THRESHOLD = 20
LEADCAR_TO_OBSTACLE = 15
LEADCAR_BREAKING_THRESHOLD = 11


#EGO BEHAVIOR
behavior EgoBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	while True:
		if ((distance from ego to leadCar) > EGO_BRAKING_THRESHOLD):
			take actions.SetThrottleAction(THROTTLE_ACTION)
		else:
			take actions.SetBrakeAction(BRAKE_ACTION)

#OTHER BEHAVIOR
behavior LeadingCarBehavior():
	take actions.SetManualFirstGearShiftAction()
	take actions.SetManualGearShiftAction(False)

	while True:
		if ((distance from leadCar to obstacle) > LEADCAR_BREAKING_THRESHOLD):
			take actions.SetThrottleAction(THROTTLE_ACTION)
		else:
			take actions.SetBrakeAction(BRAKE_ACTION)

#GEOMETRY
laneSecs = []
for lane in network.lanes:
	for sec in lane.sections:
		laneSecs.append(sec)

initLaneSec = laneSecs[10] # NOTE: Hard coded for testing


#PLACEMENT
leadCar = Car on initLaneSec.centerline,
	with speed 0,
	with behavior LeadingCarBehavior

ego = Car following roadDirection from leadCar by EGO_TO_LEADCAR, 
	with speed 0,
	with behavior EgoBehavior

obstacle = Car following roadDirection from leadCar by LEADCAR_TO_OBSTACLE
