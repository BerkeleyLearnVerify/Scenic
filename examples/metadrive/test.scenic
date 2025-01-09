param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model
import numpy as np

# ego = new Car

# car heading North
ego = new Car with heading 0

# 90 deg should be west
# ego = new Car with heading 90 deg

# ego = new Car at (0, 0), with regionContainedIn everywhere
