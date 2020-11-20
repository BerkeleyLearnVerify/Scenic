""" Scenario Description
Example traffic lights management
"""

## SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town01'
model scenic.domains.driving.model

## CONSTANTS
EGO_MODEL = "vehicle.lincoln.mkz2017"
EGO_SPEED = 10

## MONITORS
#monitor TrafficLights:
#    while True:
#        if withinDistanceToTrafficLight(ego, 100):
#            setClosestTrafficLightStatus(ego, "green")
#        wait

## DEFINING BEHAVIORS
#behavior EgoBehavior(speed=10):
#    do FollowLaneBehavior(speed)
    
behavior EgoBehaviorTL(speed=10):
    try:
        do FollowLaneBehavior(speed)
    interrupt when withinDistanceToRedYellowTrafficLight(self, 15):
        take SetBrakeAction(1.0)

#behavior EgoBehaviorSetTL(speed=10):
#    changed = False
#    try:
#        do FollowLaneBehavior(speed)
#    interrupt when withinDistanceToTrafficLight(self, 15):
#        take SetTrafficLightAction("green")
#        changed = True

## DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py

# make sure to put '*' to uniformly randomly select from all elements of the list, 'lanes'
lane = Uniform(*network.lanes)

ego = Car on lane.centerline,
    with blueprint EGO_MODEL,
    with behavior EgoBehaviorTL(EGO_SPEED)

require (distance from ego to intersection) < 50
require (ego._intersection is None)
