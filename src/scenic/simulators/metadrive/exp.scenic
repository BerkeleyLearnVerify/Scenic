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
param map = localPath(root_user + '/ScenicGym/assets/maps/CARLA/Town04.xodr')
param carla_map = 'Town04'
param time_step = 1.0/10
param camera_position = (311, 255, 0)
model scenic.domains.driving.model

success_reward = 10.0
driving_reward = 1.0
crash_penalty = -5.0
out_of_road_penalty = -5.0
speed_reward = 0.1

car1_dir = 180
car2_dir = -90

# moves in -y direction, or in +y???
# or (311, 255, 0)
ego = new Car on (311, 255, 0), facing car1_dir deg,
                                with name "agent0",
                                with goal (311, 235, 0),
                                with behavior FollowLaneBehavior(),
# moves in +x
# (or 300, 246 on town 4 map)
car2 = new Car on (300, 246, 0), facing car2_dir deg, 
                                with name "agent1",
                                with goal (320, 246, 0),
                                with behavior FollowLaneBehavior(),

monitor Reward(car1, car2):
    # TODO, will there be race conditions when adding to the reward?
    # TODO check if car dimensions are right
    # TODO the timing of the variable setting in the grand scheme of things needs to be ascertained

    done = False
    last_long_1 = car1.position[1]
    last_long_2 = car2.position[0]

    drive_dir_1 = car1_dir * pi/180
    drive_dir_2 = car2_dir * pi/180

    while True:
        # print(f"car1 pos: {car1.position}")
        # print(f"car2 pos: {car2.position}")
        # print(f"Monitored reward: {car1.reward, car2.reward}")
        if done:
            terminate

        # print(f"CAR1 CRASHED: {car1.metaDriveActor.crash_vehicle}")
        # print(f"CAR2 CRASHED: {car2.metaDriveActor.crash_vehicle}")
        
        car1.zero_reward()
        car2.zero_reward()
        
        current_long_1 = car1.position[1] 
        current_long_2 = car2.position[0] 

        car1.add_reward(driving_reward * (-1) * (current_long_1 - last_long_1)) # times -1 since car1 is going in -y direction
        car2.add_reward(driving_reward * (current_long_2 - last_long_2))
        # print(f"added lane reward: {car1.reward, car2.reward}")
        
        last_long_1 = current_long_1
        last_long_2 = current_long_2

        car1.add_reward(speed_reward * car1.speed_km_h/car1.max_speed_km_h) # TODO what is the max speed?
        car2.add_reward(speed_reward * car2.speed_km_h/car2.max_speed_km_h) # TODO what is the max speed?
        # print(f"added speed reward: {car1.reward, car2.reward}")
        
        angle_rescale = lambda angle: angle + 2 * pi if angle < 0 else angle # makes sure angles are in [0, 2*pi)

        car1.add_reward(-abs(angle_rescale(car1.yaw) - angle_rescale(drive_dir_1))) # "normalized" facing reward?
        car2.add_reward(-abs(angle_rescale(car2.yaw) - angle_rescale(drive_dir_2)))
        # print(f"added ori reward: {car1.reward, car2.reward}")
        
        # FIXME this if block is flawed...maybe should be two separate if blocks?
        if (distance from car1 to car1.goal) < 1 or \
                (distance from car2 to car2.goal) < 1:
            # print(f"SUCCESS")
            car1.add_reward(success_reward)
            car2.add_reward(success_reward)
            done = True

        # print(f"cond 3: {done}")
        # done = False
        # TODO use metadrive's own collision things?
        if car1.metaDriveActor.crash_vehicle or car2.metaDriveActor.crash_vehicle:
            # print(f"CRASHED")
            car1.add_reward(crash_penalty)
            car2.add_reward(crash_penalty)
            done = True

        # print(f"cond 4: {done}")

        # if done:
            # terminate
            # print(f"we DONE {done}")
            # pass
        # done = False

        wait
   

require monitor Reward(ego, car2)
