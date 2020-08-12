from scenic.simulators.carla.map import loadLocalNetwork
loadLocalNetwork(__file__, '../../formats/opendrive/maps/CARLA/Town01.xodr')
from scenic.simulators.carla.model import *

ego = Car in intersection

ego = Car on ego.lane.predecessor

Pedestrian on visible sidewalk

third = Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
