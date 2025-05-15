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

# behavior HumanNav(x=0, y=0, z=0):
    # for _ in range(100):
        # take HumanoidNavAction(x, y, z)
    # terminate
behavior WaitHumanNav(x=0, y=0, z=0):
    for _ in range(70):
        wait
    do HumanNav(x=x, y=y, z=z)
# param EGO_Y = VerifaiRange(-5.6, -5.4)

human = new Female_0 at (-1.5, -5.5, 0), with behavior HumanNav(x=-1.5, y=-3.5, z=0)
# human = new Female_0 at (-1.5, -5.5, 0), with behavior WaitHumanNav(x=-1.5, y=-3.5, z=0)
# ego = new Female_0 at (-1.5, -5.5, 0), with behavior HumanNav(x=-1.5, y=-3.5, z=0)
# ego = new Female_0 at (-1.5, Range(-5.9, -5.4), 0), with behavior HumanNav(x=-1.5, y=-3.5, z=0)
# ego = new Female_0 at (-1.5, EGO_Y, 0), with behavior HumanNav(x=-1.5, y=-3.5, z=0)
# ego = new SpotRobot at (-1.5, -6.5, 0), with behavior MoveSpotArm()
# spot = new SpotRobot at (-1.5, -6.5, 0)
spot = new SpotRobot at (-1.5, -6.5, 0), with behavior RobotNav(x=-1.5, y=-4.0, z=0)
# human = new Female_0 at (-1.5, -5.5, 0), with behavior HumanNav(x=-1.5, y=-3.5, z=0)
# ego = new SpotRobot at (-1.5, -6.5, 0), with behavior MoveSpotArm()

# record (distance from human to ego) as dist
# record (distance from ego to spot) as dist
# record (distance from ego to spot) as dist
