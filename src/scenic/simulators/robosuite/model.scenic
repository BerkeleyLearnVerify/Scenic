# src/scenic/simulators/robosuite/model.scenic

"""Scenic world model for RoboSuite - Custom Environments Only."""

from .simulator import RobosuiteSimulator, SetJointPositions, OSCPositionAction

# Global parameters with defaults matching Robosuite's defaults
param env_config = {}
param controller_config = None
param camera_view = None
param render = True
param real_time = True
param speed = 1.0
param lite_physics = None  # None = use Robosuite default (True)

# Simulator - no more use_environment parameter
simulator RobosuiteSimulator(
    render=globalParameters.render,
    real_time=globalParameters.real_time,
    speed=globalParameters.speed,
    env_config=globalParameters.env_config,
    controller_config=globalParameters.controller_config,
    camera_view=globalParameters.camera_view,
    lite_physics=globalParameters.lite_physics
)

# Default values dictionary
DEFAULTS = {
    # Object properties
    'object_size': 0.03,
    'density': 1000,
    'friction': (1.0, 0.005, 0.0001),
    'solref': (0.02, 1.0),
    'solimp': (0.9, 0.95, 0.001, 0.5, 2.0),
    'default_color': (0.5, 0.5, 0.5, 1.0),
    
    # Arena properties
    'table_height': 0.8,
    'table_width': 0.8,
    'table_length': 0.8,
    'table_thickness': 0.05,
    
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

# Base classes
class RoboSuiteObject(Object):
    """Base class for all RoboSuite objects."""
    density: DEFAULTS['density']
    friction: DEFAULTS['friction']
    solref: DEFAULTS['solref']
    solimp: DEFAULTS['solimp']
    shape: BoxShape()
    allowCollisions: True

# Table for arena setup
class Table(RoboSuiteObject):
    """Table in environment."""
    isTable: True
    width: DEFAULTS['table_width']
    length: DEFAULTS['table_length']
    height: DEFAULTS['table_thickness']
    position: (0, 0, DEFAULTS['table_height'])

# Base class for manipulable objects
class ManipulationObject(RoboSuiteObject):
    """Base class for objects that can be manipulated."""
    color: DEFAULTS['default_color']

# MJCF Custom Object
class MJCFObject(ManipulationObject):
    """Custom object defined by MJCF XML."""
    objectType: "MJCF"
    mjcf_xml: ""  # XML string or path to XML file
    mjcf_name: "custom_object"  # Name for the object in the scene

# Primitive shape objects (matching RoboSuite's naming)
class Box(ManipulationObject):
    """Box object."""
    objectType: "Box"
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']

class Ball(ManipulationObject):
    """Ball/sphere object."""
    objectType: "Ball"
    radius: DEFAULTS['object_size']
    width: DEFAULTS['object_size'] * 2
    length: DEFAULTS['object_size'] * 2
    height: DEFAULTS['object_size'] * 2

class Cylinder(ManipulationObject):
    """Cylinder object."""
    objectType: "Cylinder"
    width: DEFAULTS['object_size'] * 2
    length: DEFAULTS['object_size'] * 2
    height: DEFAULTS['object_size'] * 4

class Capsule(ManipulationObject):
    """Capsule object."""
    objectType: "Capsule"
    width: DEFAULTS['object_size'] * 1.5
    length: DEFAULTS['object_size'] * 1.5
    height: DEFAULTS['object_size'] * 3

# Complex objects (matching RoboSuite's naming)
class Milk(ManipulationObject):
    """Milk carton object."""
    objectType: "Milk"

class Cereal(ManipulationObject):
    """Cereal box object."""
    objectType: "Cereal"

class Can(ManipulationObject):
    """Can object."""
    objectType: "Can"

class Bread(ManipulationObject):
    """Bread object."""
    objectType: "Bread"

class Bottle(ManipulationObject):
    """Bottle object."""
    objectType: "Bottle"

class Hammer(ManipulationObject):
    """Hammer object."""
    objectType: "Hammer"

class SquareNut(ManipulationObject):
    """Square nut object."""
    objectType: "SquareNut"

class RoundNut(ManipulationObject):
    """Round nut object."""
    objectType: "RoundNut"

# Robots (matching RoboSuite's naming)
class Robot(RoboSuiteObject):
    """Base robot class."""
    robot_type: "Panda"
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

class Panda(Robot):
    """Franka Emika Panda robot."""
    robot_type: "Panda"
    gripper_type: "PandaGripper"

class UR5e(Robot):
    """Universal Robots UR5e."""
    robot_type: "UR5e"
    gripper_type: "Robotiq85Gripper"

class Sawyer(Robot):
    """Rethink Robotics Sawyer."""
    robot_type: "Sawyer"
    gripper_type: "RethinkGripper"

class Jaco(Robot):
    """Kinova Jaco robot."""
    robot_type: "Jaco"
    gripper_type: "JacoThreeFingerGripper"

class IIWA(Robot):
    """KUKA IIWA robot."""
    robot_type: "IIWA"
    gripper_type: "Robotiq140Gripper"

# Behavior Library
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