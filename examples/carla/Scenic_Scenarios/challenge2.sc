from scenic.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Desktop/Carla/Dynamic-Scenic/Scenic-devel-099/examples/carla/OpenDrive/Town01.xodr')

param map = localPath('../OpenDrive/Town01.xodr')
param carla_map = 'Town01'

from scenic.domains.driving.behaviors import *
# from scenic.simulators.carla.model import *

model scenic.domains.driving.model
import scenic.simulators.carla.actions as actions

"""
Leading vehicle decelerates suddently due to an obstacle and 
ego-vehicle must react, performing an emergency brake or an avoidance maneuver.
Based on 2019 Carla Challenge Traffic Scenario 02.
"""

#CONSTANTS
EGO_SPEED = 10
LEAD_CAR_SPEED = 10

BRAKE_ACTION = 1.0
THROTTLE_ACTION = 0.6

EGO_TO_LEADCAR = -30
EGO_BRAKING_THRESHOLD = 15

LEADCAR_TO_OBSTACLE = -20
LEADCAR_BRAKING_THRESHOLD = 15


#EGO BEHAVIOR
behavior EgoBehavior(speed=10):
	
	try: 
		FollowLaneBehavior(speed)

	# interrupt when ((distance from self.position to leadCar) < EGO_BRAKING_THRESHOLD):
	interrupt when distanceToAnyCars(self, EGO_BRAKING_THRESHOLD):
		#print("car in front within distance")
		take actions.SetBrakeAction(BRAKE_ACTION)

#OTHER BEHAVIOR
behavior LeadingCarBehavior(speed=10):

	try: 
		FollowLaneBehavior(speed)

	# interrupt when ((distance from self.position to obstacle) < LEADCAR_BRAKING_THRESHOLD):
	interrupt when distanceToAnyCars(self, LEADCAR_BRAKING_THRESHOLD):
		take actions.SetBrakeAction(BRAKE_ACTION)

#GEOMETRY
lane = Uniform(*network.lanes)
lanesec = Uniform(*lane.sections)

#PLACEMENT
obstacle = Trash on lanesec.centerline

leadCar = Car following roadDirection from obstacle by LEADCAR_TO_OBSTACLE,
	with behavior LeadingCarBehavior(LEAD_CAR_SPEED)

ego = Car following roadDirection from leadCar by EGO_TO_LEADCAR,
	with behavior EgoBehavior(EGO_SPEED)
