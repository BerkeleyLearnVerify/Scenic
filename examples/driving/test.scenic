param map = localPath('../../assets/maps/CARLA/Town05.xodr')
model scenic.simulators.metadrive.model



behavior EgoBehavior():
    take SetSpeedAction(1)

ego = new Car at (0, 0),
    with behavior EgoBehavior()
