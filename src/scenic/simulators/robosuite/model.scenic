# src/scenic/simulators/robosuite/model.scenic

"""Scenic world model for RoboSuite simulator."""

from .simulator import RobosuiteSimulator

# Simulator
simulator RobosuiteSimulator(render=True, real_time=False, speed=1.0)

# Global parameters
param use_environment = None
param env_config = {}
param controller_config = None

# Base class
class RoboSuiteObject(Object):
    density: 1000
    friction: (1.0, 0.005, 0.0001)
    solref: (0.02, 1.0)
    solimp: (0.9, 0.95, 0.001, 0.5, 2.0)

# Objects
class XMLObject(RoboSuiteObject):
    """Object from XML file or string."""
    xml_path: None
    xml_string: None
    material: {}

class Cube(RoboSuiteObject):
    width: 0.05
    length: 0.05
    height: 0.05
    color: (0.8, 0.2, 0.2)

class Ball(RoboSuiteObject):
    width: 0.05
    color: (0.2, 0.8, 0.2)

class Cylinder(RoboSuiteObject):
    width: 0.05
    height: 0.1
    color: (0.2, 0.2, 0.8)

# Robots
class Robot(RoboSuiteObject):
    robot_type: "Panda"
    gripper_type: "default"
    controller_config: None
    initial_qpos: None
    base_type: "default"
    width: 0.2
    length: 0.2
    height: 0.5
    
    joint_positions[dynamic]: []
    eef_pos[dynamic]: [0, 0, 0]
    gripper_state[dynamic]: [0, 0]

class PandaRobot(Robot):
    robot_type: "Panda"
    gripper_type: "PandaGripper"
    initial_qpos: [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04]
    height: 0.9

class SawyerRobot(Robot):
    robot_type: "Sawyer"
    gripper_type: "RethinkGripper"
    initial_qpos: [0, -0.785, 0, 1.571, 0, -0.785, 0]
    height: 1.0

# Arenas
class Arena(RoboSuiteObject):
    pass

class EmptyArena(Arena):
    pass

class TableArena(Arena):
    table_height: 0.8
    table_width: 1.0
    table_length: 0.8

class BinsArena(Arena):
    bin_size: 0.3
    bin_height: 0.1

class PegsArena(Arena):
    board_size: 0.3
    peg_radius: 0.015

class WipeArena(Arena):
    table_height: 0.8
    marker_size: 0.08

class CustomArena(Arena):
    xml_string: None
    xml_path: None
    objects: []

# Furniture
class Table(RoboSuiteObject):
    width: 1.0
    length: 0.8
    height: 0.8

class PositionableTable(RoboSuiteObject):
    width: 1.0
    length: 0.8
    height: 0.8

# Actions
import scenic.core.dynamics as dynamics

class SetJointPositions(dynamics.Action):
    """Set robot joint positions."""
    def __init__(self, positions):
        self.positions = positions
    
    def applyTo(self, agent, sim):
        pass

class OSCPositionAction(dynamics.Action):
    """Operational Space Control for end-effector."""
    def __init__(self, position_delta=None, orientation_delta=None, gripper=None):
        self.position_delta = position_delta
        self.orientation_delta = orientation_delta
        self.gripper = gripper
    
    def applyTo(self, agent, sim):
        pass

# Behaviors
behavior RobustLiftBehavior():
    """Pick and place behavior using visual servoing."""
    wait; wait  # Initialize
    
    sim = simulation()
    
    # Open gripper
    for i in range(20):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=-1)
    
    # Move above object
    for step in range(100):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait; continue
        
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        target = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.1]
        error = [target[i] - eef_pos[i] for i in range(3)]
        
        if sum(e**2 for e in error)**0.5 < 0.02:
            break
        
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Move down
    for step in range(80):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait; continue
        
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        target = [cube_pos[0], cube_pos[1], cube_pos[2] - 0.02]
        error = [target[i] - eef_pos[i] for i in range(3)]
        
        if sum(e**2 for e in error)**0.5 < 0.01:
            break
        
        delta = [max(-0.2, min(0.2, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)
    
    # Close gripper
    for i in range(30):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    # Lift
    for step in range(100):
        obs = sim._current_obs
        if not obs or 'cube_pos' not in obs or 'robot0_eef_pos' not in obs:
            wait; continue
        
        cube_pos = obs['cube_pos']
        eef_pos = obs['robot0_eef_pos']
        target = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.15]
        error = [target[i] - eef_pos[i] for i in range(3)]
        
        if sum(e**2 for e in error)**0.5 < 0.02:
            break
        
        delta = [max(-0.3, min(0.3, e * 3)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=1)
    
    # Hold
    for i in range(100):
        take OSCPositionAction(position_delta=[0, 0, 0], gripper=1)
    
    success = sim.checkSuccess()
    
    while True:
        wait

behavior SimpleLiftBehavior():
    do RobustLiftBehavior()