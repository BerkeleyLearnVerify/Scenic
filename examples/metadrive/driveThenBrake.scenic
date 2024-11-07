param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model

behavior DriveWithThrottle():
    while True:
        take SetThrottleAction(1)

behavior Brake():
    while True:
        take SetThrottleAction(0), SetBrakeAction(1)

behavior DriveThenBrake():
    do DriveWithThrottle() for 30 steps
    do Brake() for 10 steps

ego = new Car with behavior DriveThenBrake