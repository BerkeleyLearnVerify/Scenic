'''
To run this file: 
    scenic examples/carla/car.scenic --2d --model scenic.simulators.carla.model --simulate
'''
param map = localPath('../../assets/maps/CARLA/Town01.xodr')
model scenic.simulators.carla.model

ego = new Car
