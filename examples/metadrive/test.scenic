param map = localPath('../../assets/maps/CARLA/Town01.xodr')
param sumo_map = localPath('../../assets/maps/CARLA/Town01.net.xml')
model scenic.simulators.metadrive.model
import numpy as np

# ego = new Car

# car heading North
# ego = new Car with heading np.pi/2

ego = new Car facing 0 deg
