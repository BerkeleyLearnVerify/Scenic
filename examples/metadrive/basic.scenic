param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model


behavior ApplyThrottle():
    while True:  
        take SetThrottleAction(0.5)

ego = new Car at (0,0), with regionContainedIn everywhere, with behavior ApplyThrottle

