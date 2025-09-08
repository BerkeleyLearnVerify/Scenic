param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param time_step = 1.0/10

model scenic.simulators.metadrive.model

behavior WaitForThenDrive():
    wait for 20 steps
    do FollowLaneBehavior()

behavior WaitUntilThenDrive():
    wait until distance from self to ego < 40
    do FollowLaneBehavior()

ego = new Car with behavior WaitForThenDrive()

frontCar = new Car ahead of ego by 50,
                    with behavior WaitUntilThenDrive()

terminate after 100 steps
