'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.carla.model --simulate
'''
import os
import random
root_user = os.path.expanduser("~")
param map = localPath(root_user + '/ScenicGym/assets/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
param time_step = 1.0/10
param camera_position = (340, -208, 0)
model scenic.domains.driving.model


ego = new Car on (340, -208, 0), facing 0 deg,
                                with name "agent0",
                                # with behavior FollowLaneBehavior(),

car2 = new Car on (325, -200, 0), facing -90 deg, 
                                with name "agent1",
                                with behavior FollowLaneBehavior(),
