Tutorial on Dynamic Scenario
****************************

Trying Some Examples
====================
An interface between a Scenic program and a simulator is instantiated with in a server/client communication over a websocket. 
Open a terminal and instantiate a simulator. On another terminal, instantiate the scenic program using the following command::

	python -m scenic --time 100 --count 10 -S examples/carla/ScenicScenario/



Behavior Library
================
One of the strengths of Scenic is its ability to reuse functions, behaviors, and scenarios, thereby expediting the process to write more intricate scenarios. We provide a library of behaviors to help users how to take advantage of this re-usability aspect of Scenic. 

..function:: FollowLaneBehavior(target_speed, laneToFollow)
	This behavior follows the lane that the vehicle taking this action is on. As the vehicle approaches an intersection, by default, the vehicle takes the
	straight path. However, if the straight path does not exist, then it uniformly randomly choose from possible turn maneuvers and follows that lane. 
	This behavior has no termination condition specified. 

	:param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
	:param laneToFollow: If the lane to follow is different from the lane that the vehicle is on, this parameter can be used to specify that lane. By default, this variable will be set to None, which means that the vehicle will follow the lane that it is currently on. 
	:return: None


..function:: FollowTrajectoryBehavior(target_speed, trajectory)
	This behavior will follow the trajectory that user provides via ``trajectory`` input argument. The user must provide a list of lanes to follow. 

	:param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
	:param trajectory: It is a list of sequential lanes to track, from the lane that the vehicle is initially on to the lane it should end up on.  
	:return: None


..function:: WalkForwardBehavior()
	This behavior is specifically for a pedestrian. It will uniformly randomly choose either end of the sidewalk that the pedestrian is on, and have the pedestrian walk towards the endpoint. 

	:return: None

..function:: distanceToAnyObjs(vehicle, thresholdDistance)
	This is not a behavior, but a function. It is a function which computes a distance of the vehicle to any other objects (i.e. instantiated in the scenic program) that share the same lane as the vehicle and are in front of the vehicle. If there exists any vehicle within the threshold distance, the function outputs False, otherwise, True. 

	:param vehicle: the vehicle with respect to which the distance of other Scenic instantiated objects will be computed
 	:param thresholdDistance: the criterion to determine the boolean output
 	:return: boolean
 


