import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
import math
import time
import numpy as np
behavior GoRel(x=0, y=0, z=0, rot=0, num_steps=100):
    # agent = simulation().sim.agents_mgr[self._agent_id].articulated_agent
    dx = x/num_steps
    dy = y/num_steps
    dz = z/num_steps
    
    # print('taking action!')
    for _ in range(num_steps):
        # print('taking action!')
        take GoRelDeltaAction(self, dx, dy, dz)
        
        print(self.position)
        # take GoRelDeltaAction()
    if self._articulated_agent_type == 'FetchRobot':
        terminate

behavior MoveAndBack(x=0, y=0, z=0, num_steps=100):
    start_pos = self.position
    try:
        do GoRel(x=x, y=y, z=z, num_steps=100)
    interrupt when (self.distanceToClosest(KinematicHumanoid) < 1.5):
        try:
            do GoRel(x=-x/2, y=-y/2, z=-z/2, num_steps=100)
        interrupt when all(np.isclose(np.array((self.position - start_pos)), np.zeros(3), atol=0.1)):
            terminate
        terminate
    terminate

bed = RectangularRegion((0, -6.0, 0.4), 1.57, 1.0, 1.0)
ego = new FetchRobot at (Range(-1.4, -1.2), Range(-5.5, -5.0), 0), with yaw Range(-35, 0) deg, with behavior MoveAndBack(x=3, y=2, num_steps=100)
human = new Female_0 at (Range(-1.4, 1.5), Range(-4.5, -2.5), 0,), with yaw Range(90, 180) deg
# master_chef = new MasterChef at (-3.5, 0, -1.5)
# tennis_ball = new TennisBall at (-4.5, 0, -1.5)
master_chef = new MasterChef on bed
# require distance from master_chef to ego 
require distance from human to ego > 2
