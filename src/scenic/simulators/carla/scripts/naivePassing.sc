import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.models.model import *


behavior SlowCarBehavior():
	take actions.SetThrottleAction(0.3)

behavior EgoBehavior():
	take actions.SetThrottleAction(0.6)
	for _ in range(30):
		take None
	print('Ego changing lanes left')
	# lane change left
	take actions.SetSteerAction(-0.3)
	for _ in range(5):
		take None
	take actions.SetSteerAction(0.2)
	for _ in range(6):
		take None
	take actions.SetSteerAction(0)
	for _ in range(30):
		take None
	take actions.SetThrottleAction(0.4)
	print('Ego changing lanes right')
	# lane change right
	take actions.SetSteerAction(0.3)
	for _ in range(3):
		take None
	take actions.SetSteerAction(-0.3)
	for _ in range(4):
		take None
	take actions.SetSteerAction(0)


slowCar = Car with behavior SlowCarBehavior
ego = Car behind slowCar by 20, with behavior EgoBehavior
