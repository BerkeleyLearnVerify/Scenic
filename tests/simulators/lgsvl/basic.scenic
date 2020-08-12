from scenic.simulators.lgsvl.map import loadLocalNetwork
loadLocalNetwork(__file__, '../../formats/opendrive/maps/LGSVL/cubetown.xodr')
from scenic.simulators.lgsvl.model import *

ego = Car in intersection

ego = Car on ego.lane.successor

Pedestrian on visible intersection.boundary,
    with regionContainedIn None

third = Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
