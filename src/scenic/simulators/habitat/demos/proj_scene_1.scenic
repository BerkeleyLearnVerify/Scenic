import magnum as mn
import numpy as np
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
from scenic.simulators.habitat.utils import scenic_to_habitat_map
from scenic.core.vectors import Vector
import math
import time

behavior NavToObj(obj):
    x, y, z = obj.position
    do RobotNav(x=x, y=y, z=z)

# bed = RectangularRegion((0, -6.0, 0.63), 1.57, 1.0, 1.0)
bed = RectangularRegion((0.3, -6.0, 0.63), 0, 1.0, 1.0) # final defined bed width
# master_chef = new MasterChef on bed
box = new GelatinBox on bed
# master_chef = new MasterChef on (1.5, -6.0, 0.65)
# master_chef = new MasterChef on (0.80, -6.0, 0.65)
# master_chef = new MasterChef on (0.80, -6.5, 0.65)
# master_chef = new MasterChef on (0.0, -6.0, 0.63)
# master_chef = new MasterChef on (-0.2, -6.0, 0.63)
human = new Female_0 at (-1.5, -5.5, 0), with yaw 90 deg,with behavior HumanNav(x=-1.5, y=-3.0, z=0)
ego = new SpotRobot at (-5.0, -3.0, 0), with yaw 90 deg, with behavior RobotNav(x=-1.5, y=-3.7, z=0)
# ego = new SpotRobot at (-5.0, -3.0, 0), with yaw 90 deg, with behavior RobotNav(box)
human_2 = new Female_0 at (Range(0, 0.8), -7.5, 0), with yaw 90 deg

require distance from box to human_2 < 2.0
