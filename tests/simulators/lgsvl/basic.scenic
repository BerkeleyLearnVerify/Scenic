param map = localPath('../../formats/opendrive/maps/LGSVL/cubetown.xodr')
param lgsvl_map = 'CubeTown'
from scenic.simulators.lgsvl.model import *

ego = Car in intersection

ego = Car on ego.lane.successor

Pedestrian on visible intersection.boundary,
    with regionContainedIn None

third = Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
