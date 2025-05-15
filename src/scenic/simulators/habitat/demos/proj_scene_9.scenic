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


behavior SpotHandOver():
    # raise_pos = np.array([0.0, -3.14, 0.00, 3.14, 0.0, 0.0, 0.0]) # initial pos
    raise_pos = np.array([0.0, -3.14, 0.00, 1.57, 0.0, 0.0, 0.0]) # forarm raise
    do MoveToJointAngles(raise_pos)

    raise_pos = [0.0, -1.3, 0.0, 1.8, 0.0, 0.0, 0.0] # shoulder raise
    do MoveToJointAngles(raise_pos)
    


behavior SpotPickUp():
    raise_pos = np.array([0.0, -3.14, 0.00, 1.57, 0.0, 0.0, 0.0]) # forarm raise
    do MoveToJointAngles(raise_pos)
    
    raise_pos = [0.0, -1.0, 0.0, 1.57, 0.0, 0.0, 0.0] # shoulder raise
    do MoveToJointAngles(raise_pos)
    
    spot_ee_pos = self.ee_pos
    box_pos = box.position
    diff_pos = box_pos - spot_ee_pos
    diff_norm = np.linalg.norm(np.array([diff_pos[0], diff_pos[1], diff_pos[2]]))  
    print(f"pos_difference norm: {diff_norm}")
    if diff_norm < 0.25:
        take SnapToObjectAction(box)

    raise_pos = np.array([0.0, -3.14, 0.00, 3.14, 0.0, 0.0, 0.0]) # retract_arm
    do MoveToJointAngles(raise_pos, steps=50)



behavior NavToHuman():
    position = ego.position
    x, y, z = position[0], position[1], position[2]
    do RobotNav(x, y, z)

behavior GrabAndNav():
    do SpotPickUp()
    for _ in range(50):
        wait
    do NavToHuman()
    do SpotHandOver()
    terminate

behavior MoveToJointAngles(joint_angles, steps=50):
    start_pos = np.array(self._articulated_agent.arm_joint_pos)
    delta_pos = (joint_angles - start_pos)/steps
    for _ in range(steps):
        new_pos = list(start_pos + delta_pos)
        take SpotMoveArmAction(arm_ctrl_angles=new_pos)
        start_pos = np.array(self._articulated_agent.arm_joint_pos)
        
behavior ReachHandAndWalk(walk_position, reach_position):
    try:
        reach_x = reach_position[0]
        reach_y = reach_position[1]
        reach_z = reach_position[2]
        print('Reaching')
        do HumanReach(x=reach_x, y=reach_y, z=reach_z, index_hand=0)
        while True:
            wait

    interrupt when spot._holding_object:
        take HumanStopAction()
        walk_x = walk_position[0]
        walk_y = walk_position[1]
        walk_z = walk_position[2]

        do HumanNav(x=walk_x, y=walk_y, z=walk_z)
        # terminate
        while True:
            wait
    

behavior Traverse():
    while True:
        # x, y, z = point1
        # do RobotNav(x=x, y=y, z=z)
        do GoRel(y=1.0)
        do TurnAround()
        do GoRel(y=-1.0)
        do TurnAround()
        # x, y, z = point2
        # do RobotNav(x=x, y=y, z=z)
        

ego = new Female_0 at (-0.5, -4.8, 0), with yaw -90 deg,
                                with behavior ReachHandAndWalk((-4.5, -3.0, 0), (-0.5, -0.5, 0.5))

# bed = RectangularRegion((0.3, -6.0, 0.63), 0, 1.0, 1.0) # final defined bed width
box = new GelatinBox on (0.12, -5.5, 0.61)
spot = new SpotRobot at (-0.9, -5.5, 0), with behavior GrabAndNav()

# trav_point1 = (-3.5, Range(-2.5, -4.0), 0)
# trav_point2 = (-3.5, Range(-4.0, -5.0), 0)

fetch = new FetchRobot at (-3.7, Range(-2.5, -5.0), 0), with yaw 90 deg, with behavior Traverse()



