
param map = localPath('../../formats/opendrive/maps/CARLA/Town01.xodr')
model scenic.simulators.newtonian.driving_model

ego = Car in intersection, with behavior FollowLaneBehavior

ego = Car on ego.lane.predecessor, with behavior FollowLaneBehavior

Pedestrian on visible sidewalk

third = Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg

Object visible, with width 0.1, with length 0.1
