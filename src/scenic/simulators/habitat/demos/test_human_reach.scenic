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

    
behavior TestHumanReach(x=0, y=0, z=0):
    arr = np.array([x,y,z])
    offset = self._articulated_agent.base_transformation.transform_vector(mn.Vector3(0, 0.3, 0))
    print(f"INITIAL HAND POSE: {self._articulated_agent.ee_transform(0).translation + offset}")
    do HumanReach(x=x,  y=y, z=z)
    # for _ in range(200):
        # take HumanReachAction(x=arr[0], y=arr[1], z=arr[2], index_hand=0)

# human = new Female_0 at (-0.5, -4.8, 0), with yaw -90 deg, with behavior HumanReach(x=0, y=0, z=0)
human = new Female_0 at (-0.5, -4.8, 0), with yaw -90 deg, with behavior TestHumanReach(x=0, y=1.5, z=0)
bed = RectangularRegion((0.3, -6.0, 0.63), 0, 1.0, 1.0) # final defined bed width
box = new GelatinBox on (0.12, -5.5, 0.61)
spot = new SpotRobot at (-0.9, -5.5, 0)
