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
    # if self._articulated_agent_type == 'FetchRobot':
        terminate

behavior MoveAndBack(x=0, y=0, z=0, num_steps=100):
    try:
        do GoRel(x=x, y=y, z=z, num_steps=100)
    interrupt when (self.distanceToClosest(KinematicHumanoid) < 1.5):
        do GoRel(x=-x/2, y=-y/2, z=-z/2, num_steps=100)
        terminate

behavior HumanGo(x=0, y=0, z=0, num_steps=100):

    step_count = 0
    # pos_delta = Vector(x, y, z)
    
    # x, y, z = self.position + pos_delta
    start_position = self.position
    while step_count < num_steps and \
            not np.isclose((self.position - start_position).norm(), Vector(x, y, z).norm(), atol=0.1):
        take HumanGoEnvAction(x, y, z) # TODO temporary implementation
        print(f"Scenic position: {self.position}")
        step_count += 1

    take HumanStopAction()

    print('finished walking')
    print(f"target: {x, y, z}")
    # print(f"pos_delta: {pos_delta}")
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 3:
        wait
        t1 = time.time()
    print('finish scene')
    terminate

behavior HumanReach(x=0, y=0, z=0, index_hand=0):
    arr = np.array([x, y, z])
    arr = (arr - 0.5) * 0.1
    
    for _ in range(100):
        take HumanReachAction(x=arr[0], y=arr[1], z=arr[2], index_hand=index_hand)

behavior GoAndReach(reach_x=0, reach_y=0, reach_z=0, move_x=0, move_y=0, move_z=0, index_hand=0):
    do HumanReach(x=reach_x, y=reach_y, z=reach_z, index_hand=index_hand)
    do HumanGo(x=move_x, y=move_y, z=move_z)

behavior RobotNav(x=0, y=0, z=0):
    for _ in range(100):
        take OracleCoordAction(x, y, z)
    terminate

behavior MoveSpotArm():
    take SpotMoveArmAction(arm_ctrl_angles=[1.57, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0])
    # take SpotMoveArmAction(arm_ctrl_angles=[0.00, -3.14, 0.0, 3.00, 0.0, 0.0, 0.0]) # default
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 1.5:
        wait
        t1 = time.time()
    print('finish scene')
    take SpotMoveArmAction()

behavior HumanNav(x=0, y=0, z=0):
    for _ in range(100):
        take HumanoidNavAction(x, y, z)
    terminate
# human = new Female_0 at (-1.5, -5.5, 0), with behavior HumanGo(y=1)
# human = new Female_0 at (-1.5, -5.5, 0), with behavior GoAndReach(reach_x=1.0, reach_y=1.0, 
                                                                  # reach_z=-0.1, move_y=1, index_hand=0)

human = new Female_0 at (-1.5, -5.5, 0), with behavior HumanNav(x=-1.5, y=-3.0, z=0)
# spot = new SpotRobot at (-1.5, -6.5, 0), with behavior MoveSpotArm()
spot = new SpotRobot at (-1.5, -6.5, 0), with behavior RobotNav(x=-1.5, y=-3.7, z=0)
