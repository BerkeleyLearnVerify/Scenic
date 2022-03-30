
param map = localPath('../../formats/opendrive/maps/CARLA/Town01.xodr')
model scenic.simulators.newtonian.model

ego = Car in intersection, with behavior FollowLaneBehavior

ego = Car on ego.lane.predecessor, with behavior FollowLaneBehavior

Pedestrian on visible sidewalk

third = Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg
