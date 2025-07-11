param map = localPath('../../assets/maps/CARLA/Town10HD_Opt.xodr')
model scenic.simulators.carla.model
param carla_map = "Town10HD_Opt"
param time_step = 1.0/10


behavior DriveFullThrottle():
    while True:
        take SetThrottleAction(1)

ego = new Car at (-42, -137),
    with behavior DriveFullThrottle,
    with blueprint "vehicle.nissan.patrol"

terminate after 8 seconds