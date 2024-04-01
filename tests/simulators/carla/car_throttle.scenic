param map = localPath('../../../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

behavior Drive():
    take SetThrottleAction(0.1)
    take SetThrottleAction(0.4)
    take SetThrottleAction(0.9)

ego = new Car with behavior Drive
