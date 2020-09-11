
param map = localPath('../../formats/opendrive/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
from scenic.simulators.carla.model import *

ego = Car in intersection

ego = Car on ego.lane.predecessor

Pedestrian on visible sidewalk

third = Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
