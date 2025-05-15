import magnum as mn
model scenic.simulators.habitat.model
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.behaviors import *
from scenic.simulators.habitat.model import *
import math
import time

# behavior GoRel(x=0, y=0, z=0, rot=0, num_steps=100):
    # # agent = simulation().sim.agents_mgr[self._agent_id].articulated_agent
    # dx = x/num_steps
    # dy = y/num_steps
    # dz = z/num_steps
    
    # # print('taking action!')
    # for _ in range(num_steps):
        # # print('taking action!')
        # take GoRelDeltaAction(self, dx, dy, dz)
        
        # print(self.position)
        # # take GoRelDeltaAction()
    # if self._articulated_agent_type == 'FetchRobot':
        # terminate

# behavior MoveAndBack(x=0, y=0, z=0, num_steps=100):
    # try:
        # do GoRel(x=x, y=y, z=z, num_steps=100)
    # interrupt when (self.distanceToClosest(KinematicHumanoid) < 1.5):
        # do GoRel(x=-x/2, y=-y/2, z=-z/2, num_steps=100)
        # terminate
behavior OpenGripper():
    take OpenGripperAction()
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    take CloseGripperAction()
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    terminate
    
behavior SnapToObject(target_object):
    t0 = time.time()
    t1 = t0
    take SnapToObjectAction(target_object)
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    terminate
# ego = new FetchRobot at (Range(-6.0, -5.5), 0, Range(-1.8, -1.3)), with yaw -35 deg, with behavior MoveAndBack(x=3, z=0, num_steps=100)
# tennis_ball = new TennisBall at (-4.5, 0, -1.5)
ego = new SpotRobot at (-1.5, -5.5, 0), with yaw -35 deg, with behavior OpenGripper()
# tennis_ball = new TennisBall at (-4.5, 0, -1.5)
