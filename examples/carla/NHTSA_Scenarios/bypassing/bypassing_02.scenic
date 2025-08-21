"""
TITLE: Bypassing 02
AUTHOR: Francis Indaheng, findaheng@berkeley.edu
DESCRIPTION: Adversary vehicle performs a lane change to bypass the
slow ego vehicle before returning to its original lane.
SOURCE: NHSTA, #16

To run this file using the Carla simulator:
    scenic examples/carla/NHTSA_Scenarios/bypassing/bypassing_02.scenic --2d --model scenic.simulators.carla.model --simulate
"""

#################################
# MAP AND MODEL                 #
#################################

param map = localPath('../../../../assets/maps/CARLA/Town10HD_Opt.xodr')
param carla_map = 'Town10HD_Opt'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.nissan.patrol'

param EGO_SPEED = VerifaiRange(2, 4)

param ADV_DIST = VerifaiRange(-35, -25)
param ADV_SPEED = VerifaiRange(6, 8)

BYPASS_DIST = [25, 10]
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
egoSpawnPt = new OrientedPoint in initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

adversary = new Car following roadDirection for globalParameters.ADV_DIST,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST
require always (ego.laneSection._fasterLane is not None)
