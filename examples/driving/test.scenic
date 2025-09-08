param map = localPath('../../assets/maps/CARLA/Town05.xodr')
param time_step = 1.0/10

# model scenic.domains.driving.model
model scenic.simulators.metadrive.model

behavior waitThenDrive():
    wait for 60 seconds
    # wait for 60 steps
    do FollowLaneBehavior()

ego = new Car with behavior waitThenDrive()
