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

behavior RobustHumanNav(x=0, y=0, z=0, sample_radius=1.0):
    x, y, z, _, _, _ = scenic_to_habitat_map((x, y, z, 0, 0, 0))
    navigable = simulation().sim.pathfinder.is_navigable(np.array([x, y, z]))
    print(f"Point Navigable: {navigable}")
    if navigable:
        do HumanNav(x=x, y=y, z=z)
    else:
        new_point = simulation().sim.pathfinder.get_random_navigable_point_near(circle_center=np.array([x, y, z]),
                                                                                    radius=1.0)
        x, y, z = new_point
        print(f'new_point: {new_point}')
        do HumanNav(x=x, y=y, z=z)


bed = RectangularRegion((0.3, -6.0, 0.63), 0, 1.0, 1.0) # final defined bed width
box = new GelatinBox on bed
# human = new Female_0 at (-1.5, -5.5, 0), with yaw 90 deg,with behavior HumanNav(x=-1.5, y=-3.0, z=0)
# human = new Female_0 at (-5.0, -3.0, 0), with yaw 90 deg,with behavior HumanNav(x=-1.5, y=-4.0, z=0)
human = new Female_0 at (-5.0, -3.0, 0), with yaw 90 deg,with behavior RobustHumanNav(x=-1.5, y=-4.0, z=0)
# ego = new SpotRobot at (-5.0, -3.0, 0), with yaw 90 deg, with behavior RobotNav(x=-1.5, y=-3.7, z=0)
ego = new SpotRobot at (-1.5, -6.5, 0)
human_2 = new Female_0 at (Range(0, 0.8), -7.5, 0), with yaw 90 deg

require distance from box to human_2 < 2.0
