param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model


# behavior EgoBehavior():
#     do FollowLaneBehavior()

# ego = new Car with behavior EgoBehavior

# This also is not working
ego = new Car with behavior FollowLaneBehavior
