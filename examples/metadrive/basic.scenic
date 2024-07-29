param map = localPath('../../assets/maps/CARLA/Town01.xodr')
model scenic.simulators.metadrive.model


behavior ApplyThrottle():
    while True:  
        take SetThrottleAction(0.5)

ego = new Car on road, with behavior ApplyThrottle

