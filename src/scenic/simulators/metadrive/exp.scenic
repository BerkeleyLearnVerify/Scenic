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

success_reward = 10.0
driving_reward = 1.0
crash_penalty = -5.0
out_of_road_penalty = -5.0
speed_reward = 0.1

car1_dir = 0
car2_dir = -90
# car1_goal = () 
# behavior ComputeDriverReward():
    # while True: 
        # wait

ego = new Car on (340, -208, 0), facing car1_facing deg,
                                with name "agent0",
                                # with behavior ComputeDriveReward()
                                # with behavior FollowLaneBehavior(),

car2 = new Car on (325, -200, 0), facing car2_facing deg, 
                                with name "agent1",
                                # with behavior ComputeDriverReward()
                                # with behavior FollowLaneBehavior(),

monitor Collision(car1, car2):
    # TODO, will there be race conditions when adding to the reward?
    # TODO check if car dimensions are right
    # TODO the timing of the variable setting in the grand scheme of things needs to be ascertained
    done = False

    while True:
        car1.zero_reward()
        car2.zero_reward()

        car1.add_reward(driving_reward * ...) # TODO figure out car1/2's longitude's corresponding coordinate
        car2.add_reward(driving_reward * ...)

        car1.add_reward(speed_reward * car1.speed_km_h/car1.max_speed_km_h) # TODO what is the max speed?
        car2.add_reward(speed_reward * car2.speed_km_h/car2.max_speed_km_h) # TODO what is the max speed?

        car1.add_reward(-abs(car1.yaw - car1_dir)/pi) # "normalized" facing reward?
        car2.add_reward(-abs(car2.yaw - car2_dir)/pi)

        if not (car1 in lane):
            car1.add_reward(-out_of_road_penalty)
            done = True

        if not (car2 in lane):
            car2.add_reward(-out_of_road_penalty)
            done = True
        
        if (distance from car1 to car1.goal) < 1 or \
                (distance from car2 to car2.goal) < 1:
            car1.add_reward(success_reward)
            car2.add_reward(success_reward)
            done = True

        if car1 intersects car2:
            car1.add_reward(crash_penalty)
            car2.add_reward(crash_penalty)
            done = True
        
        if done:
            terminate
   

require monitor Collision(ego, car2)
