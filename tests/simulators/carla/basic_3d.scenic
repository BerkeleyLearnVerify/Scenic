param map = localPath('../../../assets/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
from scenic.simulators.carla.model import *

# In 3D, use "on" (not "in") for reliable spawning (applies contact offset).
ego = new Car on intersection

ego = new Car on ego.lane.predecessor

new Pedestrian on visible sidewalk

third = new Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
