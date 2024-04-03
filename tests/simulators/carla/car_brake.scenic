param map = localPath('../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
param time_step = 1.0/10

model scenic.simulators.carla.model

behavior DriveWithAppliedThrottle():
    do FollowLaneBehavior() for 2 seconds
    take SetBrakeAction(1)

ego = new Car with behavior DriveWithAppliedThrottle
record ego.speed as CarSpeed
terminate after 4 seconds
