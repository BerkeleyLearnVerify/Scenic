'''
To run this file using the MetaDrive simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.metadrive.model --simulate

To run this file using the Carla simulator:
    scenic examples/driving/badlyParkedCarPullingIn.scenic --2d --model scenic.simulators.carla.model --simulate
'''

# TODO add goal point
# TODO add goal reach reward
# TODO add driving (moving forward) reward?
# TODO add speed reward
# should there be a reward for total distance travelled foward?
# Check MetaDriveEnv for rough scale of the reward

import os
import random
from math import pi
root_user = os.path.expanduser("~")
param map = localPath(root_user + '/ScenicGym/assets/maps/CARLA/Town01.xodr')
param carla_map = 'Town01'
param time_step = 1.0/10
param camera_position = (340, -208, 0)
model scenic.domains.driving.model


collision_penalty = -5.0
out_of_lane_penalty = -5.0

car1_dir = 0
car2_dir = -90

behavior ComputeDriverReward():
    while True: 
        wait

ego = new Car on (340, -208, 0), facing car1_facing deg,
                                with name "agent0",
                                with behavior ComputeDriveReward()
                                # with behavior FollowLaneBehavior(),

car2 = new Car on (325, -200, 0), facing car2_facing deg, 
                                with name "agent1",
                                with behavior ComputeDriverReward()
                                # with behavior FollowLaneBehavior(),

monitor Collision(car1, car2):
    # TODO, will there be race conditions when adding to the reward?
    # TODO check if car dimensions are right
    # TODO the timing of the variable setting in the grand scheme of things needs to be ascertained

    while True:
        car1.add_reward(-abs(car1.yaw - car1_dir)/pi) # "normalized" facing reward?
        car2.add_reward(abs(car2.yaw - car2_dir)/pi)
        
        if (distance from car1 to car1.goal < 1) or \
                (distance from car2 to car2.goal < 1):
            car1.add_reward(10.0)
            car2.add_reward(10.0)
            terminate

        if car1 intersects car2:
            car1.add_reward(collision_penalty)
            car2.add_reward(collision_penalty)
            terminate

   

require monitor Collision(ego, car2)
