param map = localPath('../../assets/maps/CARLA/Town05.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town055.net.xml')
model scenic.simulators.metadrive.model
import numpy as np

ego = new Car

# car heading North
# ego = new Car with heading np.pi/2

# 90 deg should be west
# ego = new Car with heading 90 deg

# ego = new Car at (0, 0), with regionContainedIn everywhere
# ego = new Car at (155, -20), with regionContainedIn everywhere
# ego = new Car at (130, -5), with regionContainedIn everywhere
# ego = new Car at (47, 207), with regionContainedIn everywhere
