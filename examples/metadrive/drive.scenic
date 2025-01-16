param map = localPath('../../assets/maps/CARLA/Town07.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town07.net.xml')
model scenic.simulators.metadrive.model

# ego = new Car with behavior FollowLaneBehavior()
ego = new Car at (-88.2, -1.2), with behavior FollowLaneBehavior()
