param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model

ego = new Car with behavior FollowLaneBehavior()
# ego = new Car at (-88.2, -1.2), with behavior FollowLaneBehavior()
