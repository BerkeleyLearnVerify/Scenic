'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/pedestrianAcrossRoad.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/pedestrianAcrossRoad.scenic --2d --model scenic.simulators.carla.model --simulate
'''

param map = localPath('../../assets/maps/CARLA/Town01.xodr')

model scenic.domains.driving.model

ego = new Car

new Pedestrian on visible ego.oppositeLaneGroup.sidewalk
