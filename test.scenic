param map = localPath('./assets/maps/CARLA/Town01.xodr')
param time_step = 1.0/10
model scenic.simulators.newtonian.driving_model

ego = new Car with behavior FollowLaneBehavior

record distance to ego.lane.centerline

terminate after 5 seconds