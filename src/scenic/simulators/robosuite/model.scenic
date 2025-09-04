"""Scenic world model for RoboSuite."""

from .simulator import RobosuiteSimulator, SetJointPositions, OSCPositionAction

# Global parameters with defaults matching Robosuite's defaults
param use_environment = None
param env_config = {}
param controller_config = None
param camera_view = None
param render = True
param real_time = True  # Robosuite default
param speed = 1.0
param lite_physics = None  # None = use Robosuite default (True)

# Simulator
simulator RobosuiteSimulator(
    render=globalParameters.render,
    real_time=globalParameters.real_time,
    speed=globalParameters.speed,
    use_environment=globalParameters.use_environment,
    env_config=globalParameters.env_config,
    controller_config=globalParameters.controller_config,
    camera_view=globalParameters.camera_view,
    lite_physics=globalParameters.lite_physics
)

# Default values dictionary
DEFAULTS = {
    # Object properties
    'object_size': 0.05,
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

# Table class for custom environments
class Table(RoboSuiteObject):
    """Table in custom environment."""
    isTable: True
    width: DEFAULTS['table_width']
    length: DEFAULTS['table_length']
    height: DEFAULTS['table_thickness']
    position: (0, 0, DEFAULTS['table_height'])

# Base class for custom environment objects
class CustomObject(RoboSuiteObject):
    """Base class for custom environment objects."""
    envObjectName: None
    randomPlacement: False
    tableIndex: 0
    xRange: (-0.1, 0.1)
    yRange: (-0.1, 0.1)
    color: DEFAULTS['default_color']

# Primitive objects for custom environments
class CustomBox(CustomObject):
    """Box object for custom environments."""
    objectType: "Box"
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']

class CustomBall(CustomObject):
    """Ball object for custom environments."""
    objectType: "Ball"
    radius: DEFAULTS['object_size']
    width: DEFAULTS['object_size'] * 2  # For compatibility
    length: DEFAULTS['object_size'] * 2
    height: DEFAULTS['object_size'] * 2

class CustomCylinder(CustomObject):
    """Cylinder object for custom environments."""
    objectType: "Cylinder"
    width: DEFAULTS['object_size'] * 2  # Diameter
    length: DEFAULTS['object_size'] * 2
    height: DEFAULTS['object_size'] * 4  # Height

class CustomCapsule(CustomObject):
    """Capsule object for custom environments."""
    objectType: "Capsule"
    width: DEFAULTS['object_size'] * 1.5
    length: DEFAULTS['object_size'] * 1.5
    height: DEFAULTS['object_size'] * 3

# Complex objects for custom environments
class CustomMilk(CustomObject):
    """Milk carton for custom environments."""
    objectType: "Milk"

class CustomCereal(CustomObject):
    """Cereal box for custom environments."""
    objectType: "Cereal"

class CustomCan(CustomObject):
    """Can object for custom environments."""
    objectType: "Can"

class CustomBread(CustomObject):
    """Bread object for custom environments."""
    objectType: "Bread"

class CustomBottle(CustomObject):
    """Bottle object for custom environments."""
    objectType: "Bottle"

class CustomHammer(CustomObject):
    """Hammer object for custom environments."""
    objectType: "Hammer"

class CustomSquareNut(CustomObject):
    """Square nut for custom environments."""
    objectType: "SquareNut"

class CustomRoundNut(CustomObject):
    """Round nut for custom environments."""
    objectType: "RoundNut"

# Standard environment objects
class Cube(RoboSuiteObject):
    """Cubic object with uniform dimensions."""
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']
    color: DEFAULTS['default_color']

class EnvironmentObject(RoboSuiteObject):
    """Base class for standard environment objects."""
    envObjectName: None
    allowCollisions: True

class LiftCube(EnvironmentObject):
    """Cube in Lift environment."""
    envObjectName: "cube"
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']

class StackCubeA(EnvironmentObject):
    """First cube in Stack environment."""
    envObjectName: "cubeA"
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']

class StackCubeB(EnvironmentObject):
    """Second cube in Stack environment."""
    envObjectName: "cubeB"
    width: DEFAULTS['object_size']
    length: DEFAULTS['object_size']
    height: DEFAULTS['object_size']

# Robots
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

class PandaRobot(Robot):
    """Franka Emika Panda robot."""
    robot_type: "Panda"
    gripper_type: "PandaGripper"

class UR5eRobot(Robot):
    """Universal Robots UR5e."""
    robot_type: "UR5e"
    gripper_type: "Robotiq85Gripper"

class SawyerRobot(Robot):
    """Rethink Robotics Sawyer."""
    robot_type: "Sawyer"
    gripper_type: "RethinkGripper"

class JacoRobot(Robot):
    """Kinova Jaco robot."""
    robot_type: "Jaco"
    gripper_type: "JacoThreeFingerGripper"

class IIWARobot(Robot):
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
    
    if simulation().checkSuccess():
        terminate simulation

