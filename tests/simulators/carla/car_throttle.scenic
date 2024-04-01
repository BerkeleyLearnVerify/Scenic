param map = localPath('../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

# behavior Drive():
#     take SetThrottleAction(0.1)
#     take SetThrottleAction(0.4)
#     take SetThrottleAction(0.9)

behavior DriveWithAppliedThrottle():
    delay = Range(1, 3)
    last_stop = 0
    try:
        do FollowLaneBehavior()
    interrupt when simulation().currentTime - last_stop > delay:
        take SetThrottleAction(0.9) #for 1 seconds
        delay = Range(1, 3)
        last_stop = simulation().currentTime

ego = new Car with behavior DriveWithAppliedThrottle
