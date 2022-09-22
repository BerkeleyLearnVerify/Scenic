
param map = localPath('../../formats/opendrive/maps/CARLA/Town01.xodr')
model scenic.simulators.newtonian.driving_model

ego = new Car in intersection, with behavior FollowLaneBehavior

ego = new Car on ego.lane.predecessor, with behavior FollowLaneBehavior

new Pedestrian on visible sidewalk

third = new Car on visible ego.road
require abs((apparent heading of third) - 180 deg) <= 30 deg

new Object visible, with width 0.1, with length 0.1
