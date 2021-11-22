"""
TITLE: Bypassing 02
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Adversary vehicle performs a lane change to bypass the 
slow ego vehicle before returning to its original lane.
SOURCE: NHSTA, #16
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../../tests/formats/opendrive/maps/CARLA/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz2017'

param EGO_SPEED = VerifaiRange(2, 4)

param ADV_DIST = VerifaiRange(-25, -10)
param ADV_SPEED = VerifaiRange(7, 10)

BYPASS_DIST = [15, 10]
INIT_DIST = 50
TERM_TIME = 5

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversaryBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
	interrupt when withinDistanceToAnyObjs(self, BYPASS_DIST[0]):
		fasterLaneSec = self.laneSection.fasterLane
		do LaneChangeBehavior(
				laneSectionToSwitch=fasterLaneSec,
				target_speed=globalParameters.ADV_SPEED)
		do FollowLaneBehavior(
				target_speed=globalParameters.ADV_SPEED,
				laneToFollow=fasterLaneSec.lane) \
			until (distance to adversary) > BYPASS_DIST[1]
		slowerLaneSec = self.laneSection.slowerLane
		do LaneChangeBehavior(
				laneSectionToSwitch=slowerLaneSec,
				target_speed=globalParameters.ADV_SPEED)
		do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for TERM_TIME seconds
		terminate 

#################################
# SPATIAL RELATIONS             #
#################################

initLane = Uniform(*network.lanes)
egoSpawnPt = OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

adversary = Car following roadDirection for globalParameters.ADV_DIST,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)
