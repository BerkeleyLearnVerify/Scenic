""" Scenario Description
Based on 2019 Carla Challenge Traffic Scenario 07.
Ego-vehicle is going straight at an intersection but a crossing vehicle 
runs a red light, forcing the ego-vehicle to perform a collision avoidance maneuver.
Note: The traffic light control is not implemented yet, but it will soon be. 
"""

#SET MAP AND MODEL (i.e. definitions of all referenceable vehicle types, road library, etc)
param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

#CONSTANTS
EGO_DISTANCE_TO_INTERSECTION = Range(15, 20) * -1
ADV_DISTANCE_TO_INTERSECTION = Range(10, 15) * -1
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

## MONITORS
monitor TrafficLights:
   while True:
       if withinDistanceToTrafficLight(ego, 100):
           setClosestTrafficLightStatus(ego, "green")
       if withinDistanceToTrafficLight(crossing_car, 100):
           setClosestTrafficLightStatus(crossing_car, "red")
       wait

##DEFINING BEHAVIORS
behavior CrossingCarBehavior(trajectory):
	do FollowTrajectoryBehavior(trajectory = trajectory)
	terminate

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
		take SetBrakeAction(BRAKE_INTENSITY)

##DEFINING SPATIAL RELATIONS
# Please refer to scenic/domains/driving/roads.py how to access detailed road infrastructure
# 'network' is the 'class Network' object in roads.py
spawnAreas = []

"""The filter() is Scenic's built-in function equivalent to the following for-loop
fourWayIntersection = []
for i in network.intersections:
	if i.is4Way:
		fourWayIntersection.append(i)
"""
fourWayIntersection = filter(lambda i: i.is4Way, network.intersections)

# make sure to put '*' to uniformly randomly select from all elements of the list
intersec = Uniform(*fourWayIntersection)
startLane = Uniform(*intersec.incomingLanes)

straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, startLane.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)
ego_trajectory = [straight_maneuver.startLane, straight_maneuver.connectingLane, straight_maneuver.endLane]

conflicting_straight_maneuvers = filter(lambda i: i.type == ManeuverType.STRAIGHT, straight_maneuver.conflictingManeuvers)

csm = Uniform(*conflicting_straight_maneuvers)
crossing_startLane = csm.startLane
crossing_car_trajectory = [csm.startLane, csm.connectingLane, csm.endLane]

## OBJECT PLACEMENT
ego_spwPt = startLane.centerline[-1] # '-1' index gives startLane's center endpoint from the list of centerpoints in 'centerline'
csm_spwPt = crossing_startLane.centerline[-1]

# Set a specific vehicle model for the Truck. 
# The referenceable types of vehicles supported in carla are listed in scenic/simulators/carla/model.scenic
# For each vehicle type, the supported models are listed in scenic/simulators/carla/blueprints.scenic
ego = Car following roadDirection from ego_spwPt for EGO_DISTANCE_TO_INTERSECTION,
		with behavior EgoBehavior(trajectory = ego_trajectory)

crossing_car = Car following roadDirection from csm_spwPt for ADV_DISTANCE_TO_INTERSECTION,
				with behavior CrossingCarBehavior(crossing_car_trajectory)

