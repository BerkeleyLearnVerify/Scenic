import scenic.simulators.carla.actions as actions

from scenic.core.geometry import subtractVectors

from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('/home/carla_challenge/Downloads/Town01.xodr')

from scenic.simulators.carla.models.model import *

behavior FollowLaneBehavior():
	


ego = Car on road,
		with speed 0,
		with behavior 


parkedCar = 