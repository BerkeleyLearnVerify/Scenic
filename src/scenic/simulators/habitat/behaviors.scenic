model scenic.simulators.habitat.model
from scenic.simulators.habitat.model import *
from scenic.simulators.habitat.actions import *
from scenic.simulators.habitat.utils import scenic_to_habitat_map
import magnum as mn
import numpy as np

behavior GoToLookAt(obj):
    obj_id = obj._object_id
    object_trans = obj._managed_rigid_object.translation

    object_agent_vec = self._articulated_agent.base_pos - object_trans
    object_agent_vec.y = 0
    dist_agent_object = object_agent_vec.length()

    agent_displ = np.inf
    agent_rot = np.inf
    
    while agent_displ > 1e-9 or agent_rot > 1e-9:
        print("WALKING")
        prev_rot = self._articulated_agent.base_rot
        prev_pos = self._articulated_agent.base_pos

        take HumanoidNavLookAtAction(object_trans)

        cur_rot = self._articulated_agent.base_rot
        cur_pos = self._articulated_agent.base_pos
        agent_displ = (cur_pos - prev_pos).length()
        agent_rot = np.abs(cur_rot - prev_rot)

    # wait
    for _ in range(20):
        wait


behavior GoRel(x=0, y=0, z=0, rot=0, num_steps=100):
    # agent = simulation().sim.agents_mgr[self._agent_id].articulated_agent
    dx = x/num_steps
    dy = y/num_steps
    dz = z/num_steps

    for _ in range(num_steps):
        take GoRelDeltaAction(dx, dy, dz)

# behavior Rotate(rot_angle, num_steps=20):
    # delta_angle = self.yaw - rot_angle

behavior TurnAround(num_steps=30):
    if self._articulated_agent.base_rot > 3.14:
        angle = -3.14
    else:
        angle = 3.14

    rot_delta = angle/num_steps
    for _ in range(num_steps):
        take RotDeltaAction(rot_delta)

behavior MoveAndBack(x=0, y=0, z=0, num_steps=100):
    try:
        do GoRel(x=x, y=y, z=z, num_steps=100)
    interrupt when (self.distanceToClosest(KinematicHumanoid) < 1.5):
        do GoRel(x=-x/2, y=-y/2, z=-z/2, num_steps=100)
        # terminate

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
    # terminate

behavior HumanReach(x=0, y=0, z=0, index_hand=0, timesteps=250):
    arr = np.array([x, y, z])
    arr = (arr - 0.5) * 0.1
    
    for _ in range(timesteps):
        take HumanReachAction(x=arr[0], y=arr[1], z=arr[2], index_hand=index_hand)

behavior GoAndReach(reach_x=0, reach_y=0, reach_z=0, move_x=0, move_y=0, move_z=0, index_hand=0):
    do HumanReach(x=reach_x, y=reach_y, z=reach_z, index_hand=index_hand)
    do HumanGo(x=move_x, y=move_y, z=move_z)

behavior RobotNav(x=0, y=0, z=0):
    for _ in range(100):
        # print(f'{self.object_type} ROBOT NAVING')
        take OracleCoordAction(x, y, z)
    # terminate

behavior MoveSpotArm():
    take SpotMoveArmAction(arm_ctrl_angles=[1.57, -1.57, 0.0, 1.57, 0.0, 0.0, 0.0])
    # take SpotMoveArmAction(arm_ctrl_angles=[0.00, -3.14, 0.0, 3.00, 0.0, 0.0, 0.0]) # default
    t0 = time.time()
    t1 = t0
    while t1 - t0 < 1.5:
        wait
        t1 = time.time()
    take SpotMoveArmAction()

behavior HumanNav(x=0, y=0, z=0):
    # current_pos = self._articulated_agent.base_pos
    # self._articulated_agent.base_pos = current_pos
    for _ in range(100):
        # print("BASE POS:", self._articulated_agent.base_pos)
        take HumanoidNavAction(x, y, z)
    # terminate

