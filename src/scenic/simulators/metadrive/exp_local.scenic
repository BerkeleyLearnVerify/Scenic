# Should there be a reward for total distance travelled foward? To give a denser signal?
# but that's taken care of by the longitudinal reward

import os
import random
from math import pi
root_user = os.path.expanduser("~")
param verifaiSamplerType = 'ce'
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
ego_x = 312
car2_y = 247

param ego_y = VerifaiRange(255, 265)
param car2_x = VerifaiRange(290, 306)

# ego car is in the lane with the slight curve
# moves in -y direction, or in +y???
# (311, 255, 0) good test position
ego = new Car on (ego_x, globalParameters.ego_y, 0), facing car1_dir deg,
                                with name "agent0",
                                with goal (311, 235, 0),
                                # with behavior FollowLaneBehavior(),

# moves in +x
# (300, 246, 0) good testing position 
car2 = new Car on (globalParameters.car2_x, car2_y, 0), facing car2_dir deg, 
                                with name "agent1",
                                with goal (320, 246, 0),
                                # with behavior FollowLaneBehavior(),

monitor Reward(car1, car2):
    # TODO, will there be race conditions when adding to the reward?
    # TODO the timing of the variable setting in the grand scheme of things needs to be ascertained

    done = False
    car1_success = False
    car2_success = False

    last_long_1 = car1.position[1]
    last_long_2 = car2.position[0]

    drive_dir_1 = car1_dir * pi/180
    drive_dir_2 = car2_dir * pi/180

    while True:

        car1.zero_reward()
        car2.zero_reward()
        
        current_long_1 = car1.position[1] 
        current_long_2 = car2.position[0] 

        car1.add_reward(driving_reward * (-1) * (current_long_1 - last_long_1)) # times -1 since car1 is going in -y direction
        car2.add_reward(driving_reward * (current_long_2 - last_long_2))
        
        last_long_1 = current_long_1
        last_long_2 = current_long_2

        car1.add_reward(speed_reward * car1.speed_km_h/car1.max_speed_km_h) 
        car2.add_reward(speed_reward * car2.speed_km_h/car2.max_speed_km_h)
        
        angle_rescale = lambda angle: angle + 2 * pi if angle < 0 else angle # makes sure angles are in [0, 2*pi)

        car1.add_reward(-abs(angle_rescale(car1.yaw) - angle_rescale(drive_dir_1))/pi * 0.1)
        car2.add_reward(-abs(angle_rescale(car2.yaw) - angle_rescale(drive_dir_2))/pi * 0.1)
        
        if (distance from car1 to car1.goal) < 1:
            car1_success = True
            car1.add_reward(success_reward)

        if (distance from car2 to car2.goal) < 1:
            car2_success = True
            car2.add_reward(success_reward)

        if car1_success and car2_success:
            done = True
        
        # can do this since the cars can only crash with each other if they do crash
        if car1.metaDriveActor.crash_vehicle or car2.metaDriveActor.crash_vehicle:
            car1.add_reward(crash_penalty)
            car2.add_reward(crash_penalty)
            done = True

        if done:
            terminate
        wait
   

require monitor Reward(ego, car2)

record abs(ego.position.x - ego_x) as ego_drift
record abs(car2.position.y - car2_y) as car2_drift
