""" Scenario Description: 
Cut-in scenario
Ego is placed on a lane which has a lane to its right
The cut-in car is on the right lane heading 15 to 30 deg towards ego's lane.
"""

model scenic.simulators.carla.model

lanesWithRightLane = filter(lambda i: i._laneToRight, network.laneSections)
egoLane = Uniform(*lanesWithRightLane)

ego = Car on egoLane, 
		facing Range(-15,15) deg relative to roadDirection
cutInCar = Car offset by Range(0, 4) @ Range(0, 5),
			facing -1* Range(15, 30) deg relative to roadDirection