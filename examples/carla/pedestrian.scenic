param map = localPath('../../../CARLA/Town03.xodr')
param carla_map = 'Town03'
param address = "10.0.0.122"
model scenic.simulators.carla.model

ego = new Car
new Pedestrian on visible sidewalk
