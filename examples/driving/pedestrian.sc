from scenic.simulators.domains.driving.network import loadLocalNetwork
loadLocalNetwork(__file__, '../../tests/simulators/formats/opendrive/maps/CARLA/Town01.xodr')

from scenic.simulators.domains.driving.model import *

ego = Car

Pedestrian visible
