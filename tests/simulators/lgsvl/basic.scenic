param map = localPath('../../../assets/maps/LGSVL/cubetown.xodr')
param lgsvl_map = 'CubeTown'
from scenic.simulators.lgsvl.model import *

ego = new Car in intersection

ego = new Car on ego.lane.successor

new Pedestrian on visible intersection.boundary,
    with regionContainedIn None

third = new Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
