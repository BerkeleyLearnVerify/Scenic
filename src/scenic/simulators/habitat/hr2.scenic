import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
import math
import time

behavior GoRel(x=0, y=0, z=0, rot=0, num_steps=100):
    agent = simulation().sim.agents_mgr[self._agent_id].articulated_agent
    dx = x/num_steps
    dy = y/num_steps
    dz = z/num_steps
    
    # print('taking action!')
    for _ in range(num_steps):
        # print('taking action!')
        take GoRelDeltaAction(agent, dx, dy, dz)
        # take GoRelDeltaAction()

    terminate

ego = new FetchRobot at (-5.5,0,-1.5), with behavior GoRel(x=-1)
human = new Female_0 at (-4.5, 0, -1.5)
