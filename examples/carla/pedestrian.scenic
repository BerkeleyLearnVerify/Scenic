'''
To run this file: 
    scenic examples/carla/pedestrian.scenic --2d --model scenic.simulators.carla.model --simulate
'''
param map = localPath('../../assets/maps/CARLA/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

ego = new Car
new Pedestrian on visible sidewalk