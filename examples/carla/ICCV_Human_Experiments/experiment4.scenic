""" Scenario Description:
On a two lane road, with each lane facing opposite traffic direction,
there is a car ahead of ego by 0 to 40 meters.
On the opposite lane, there are two cars in a line, where
one car is ahead of other by 0 to 40 meters
"""
# param map = localPath('../../../tests/formats/opendrive/maps/CARLA/Town01.xodr')  # or other CARLA map that definitely works
# param carla_map = 'Town01'
model scenic.simulators.carla.model

# parse out roads with two lanes, with opposite traffic directions
ego = Car on drivableRoad, 
		facing Range(-15, 15) deg relative to roadDirection,
		with visibleDistance 50,
		with viewAngle 135 deg

point1 = OrientedPoint ahead of ego by Range(0,40) 
Car at (point1 offset by Range(-1,1) @ 0), 
		facing Range(-15, 15) deg relative to roadDirection

oppositeCar = Car offset by (Range(-10, -1), Range(0, 50)), 
		facing Range(140, 180) deg relative to ego.heading 

point2 = OrientedPoint ahead of oppositeCar by Range(0,40)
Car at (point2 offset by Range(-1,1) @ 0), 
		facing Range(-15, 15) deg relative to roadDirection


# oppositeLaneGroup = egoLaneGroup._opposite
# oppositeLane = Uniform(*oppositeLaneGroup.lanes)  
# oppositeCar = Car on visible oppositeLane,
# 				facing Range(-15,15) deg relative to roadDirection

# point2 = OrientedPoint ahead of oppositeCar by Range(20, 40)
# Car at (point2 offset by Range(-1,1) @ 0),
# 	facing Range(-15,15) deg relative to roadDirection

