"""Scenic world model for RoboSuite simulator."""

from .simulator import RobosuiteSimulator
import scenic.core.dynamics as dynamics
import numpy as np

# Global parameters with defaults
param use_environment = None
param env_config = {}
param controller_config = None
param camera_view = None
param render = True
param real_time = True
param speed = 1.0

# Simulator
simulator RobosuiteSimulator(
    render=globalParameters.render,
    real_time=globalParameters.real_time,
    speed=globalParameters.speed,
    use_environment=globalParameters.use_environment,
    env_config=globalParameters.env_config,
    controller_config=globalParameters.controller_config,
    camera_view=globalParameters.camera_view
)

# Default values dictionary
DEFAULTS = {
    # Object properties
    'object_size': 0.05,
    'density': 1000,
    'friction': (1.0, 0.005, 0.0001),
    'solref': (0.02, 1.0),
    'solimp': (0.9, 0.95, 0.001, 0.5, 2.0),
    'default_color': (0.5, 0.5, 0.5),
    
    # Arena properties
    'table_height': 0.8,
    'table_width': 1.0,
    'table_length': 0.8,
    
    # Robot properties
    'robot_width': 0.2,
    'robot_length': 0.2,
    'robot_height': 0.5,
    
    # Control parameters
    'control_gain': 3.0,
    'control_limit': 0.3,
    'position_tolerance': 0.02,
    'height_tolerance': 0.02,
    'gripper_open_steps': 20,
    'gripper_close_steps': 30,
    'max_control_steps': 100,
    'max_lift_steps': 200
}

# Base class  
class RoboSuiteObject(Object):
    density: DEFAULTS['density']
    friction: DEFAULTS['friction']
    solref: DEFAULTS['solref']
    solimp: DEFAULTS['solimp']
    shape: BoxShape()

class Cube(RoboSuiteObject):
    """Cubic object with uniform dimensions."""
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']
    color: DEFAULTS['default_color']

"""
# currently only Cube works within preconfigured env, we can't change the type of object, WIP, will add support for them soon
class Ball(RoboSuiteObject):
    """Spherical object."""
    width: DEFAULTS['object_size']
    color: DEFAULTS['default_color']

class Cylinder(RoboSuiteObject):
    """Cylindrical object."""
    width: DEFAULTS['object_size']
    height: DEFAULTS['object_size'] * 2
    color: DEFAULTS['default_color']
"""

# Environment-specific wrapper
class EnvironmentObject(RoboSuiteObject):
    """Base class for objects in RoboSuite environments."""
    envObjectName: None  
    allowCollisions: True

# Convenience classes for specific environments
class LiftCube(Cube):
    """Cube in Lift environment."""
    envObjectName: "cube"
    allowCollisions: True

class StackCubeA(Cube):
    """First cube in Stack environment."""
    envObjectName: "cubeA"
    allowCollisions: True

class StackCubeB(Cube):
    """Second cube in Stack environment."""
    envObjectName: "cubeB"
    allowCollisions: True

# Robots
class Robot(RoboSuiteObject):
    """Base robot class."""
    robot_type: "Panda"  # Default type
    gripper_type: "default"
    controller_config: None
    initial_qpos: None
    base_type: "default"
    width: DEFAULTS['robot_width']
    length: DEFAULTS['robot_length']
    height: DEFAULTS['robot_height']
    
    # Dynamic properties
    joint_positions[dynamic]: []
    eef_pos[dynamic]: [0, 0, 0]
    gripper_state[dynamic]: [0, 0]

# Generic robot templates
class PandaRobot(Robot):
    """Franka Emika Panda robot."""
    robot_type: "Panda"
    gripper_type: "PandaGripper"

class SawyerRobot(Robot):
    """Rethink Sawyer robot."""
    robot_type: "Sawyer"
    gripper_type: "RethinkGripper"

class UR5eRobot(Robot):
    """Universal Robots UR5e."""
    robot_type: "UR5e"
    gripper_type: "Robotiq85Gripper"

# Actions
class SetJointPositions(dynamics.Action):
    """Set robot joint positions.
    
    Args:
        positions: Target joint positions
    """
    def __init__(self, positions):
        self.positions = positions
    
    def applyTo(self, agent, sim):
        """Apply joint position control to the robot."""
        if hasattr(sim, 'robots') and agent in sim.robots:
            robot_idx = sim.robots.index(agent)
            if robot_idx < len(sim.robosuite_env.robots):
                action = np.array(self.positions)
                sim.pending_robot_action = action

class OSCPositionAction(dynamics.Action):
    """Operational Space Control for end-effector.
    
    Args:
        position_delta: Cartesian position change [x, y, z]
        orientation_delta: Orientation change [roll, pitch, yaw]
        gripper: Gripper command (-1=open, 1=close)
    """
    def __init__(self, position_delta=None, orientation_delta=None, gripper=None):
        self.position_delta = position_delta if position_delta else [0, 0, 0]
        self.orientation_delta = orientation_delta if orientation_delta else [0, 0, 0]
        self.gripper = gripper if gripper is not None else 0
    
    def applyTo(self, agent, sim):
        """Apply OSC control to the robot."""
        if hasattr(sim, 'robots') and agent in sim.robots:
            robot_idx = sim.robots.index(agent)
            if robot_idx < len(sim.robosuite_env.robots):
                # Build action array based on controller type
                if hasattr(sim, 'controller_type') and sim.controller_type == 'JOINT_POSITION':
                    action = np.zeros(sim.action_dim)
                    action[:3] = self.position_delta
                else:
                    # Default OSC action [position(3), orientation(3), gripper(1)]
                    action = np.zeros(7)
                    action[:3] = self.position_delta
                    action[3:6] = self.orientation_delta
                    action[6] = self.gripper
                
                sim.pending_robot_action = action

# Behavior Library - Reusable components
behavior OpenGripper(steps=DEFAULTS['gripper_open_steps']):
    """Open gripper over multiple steps."""
    for _ in range(steps):
        take OSCPositionAction(gripper=-1)

behavior CloseGripper(steps=DEFAULTS['gripper_close_steps']):
    """Close gripper over multiple steps."""
    for _ in range(steps):
        take OSCPositionAction(gripper=1)

behavior MoveToPosition(target_pos, 
                        tolerance=DEFAULTS['position_tolerance'], 
                        max_steps=DEFAULTS['max_control_steps'], 
                        gain=DEFAULTS['control_gain']):
    """Move end-effector to target position."""
    for _ in range(max_steps):
        eef_pos = self.eef_pos
        error = [target_pos[i] - eef_pos[i] for i in range(3)]
        
        if sum(e**2 for e in error)**0.5 < tolerance:
            return
        
        limit = DEFAULTS['control_limit']
        delta = [max(-limit, min(limit, e * gain)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=-1)

behavior MoveAboveObject(target_object, height_offset=0.1):
    """Move above a tracked object."""
    target_pos = [target_object.position.x, 
                  target_object.position.y, 
                  target_object.position.z + height_offset]
    do MoveToPosition(target_pos)

behavior MoveToGrasp(target_object, grasp_offset=0.02):
    """Move to grasp position for object."""
    target_pos = [target_object.position.x, 
                  target_object.position.y, 
                  target_object.position.z - grasp_offset]
    do MoveToPosition(target_pos, tolerance=0.01)

behavior LiftToHeight(target_height=1.0, max_steps=DEFAULTS['max_lift_steps']):
    """Lift to absolute height."""
    for _ in range(max_steps):
        eef_pos = self.eef_pos
        error = [0, 0, target_height - eef_pos[2]]
        
        if abs(error[2]) < DEFAULTS['height_tolerance']:
            return
        
        limit = DEFAULTS['control_limit']
        gain = DEFAULTS['control_gain']
        delta = [max(-limit, min(limit, e * gain)) for e in error]
        take OSCPositionAction(position_delta=delta, gripper=1)

# Object-aware behaviors
behavior PickObject(target_object):
    """Pick up a specific object."""
    do OpenGripper()
    do MoveAboveObject(target_object)
    do MoveToGrasp(target_object)
    do CloseGripper()

behavior PickAndLift(target_object, height=1.05):
    """Complete pick and lift for specific object."""
    do PickObject(target_object)
    do LiftToHeight(height)
    
    if simulation().checkSuccess():
        terminate simulation
