# src/scenic/simulators/robosuite/model.scenic
"""Scenic world model for RoboSuite simulator."""

from .simulator import RobosuiteSimulator

simulator RobosuiteSimulator(render=True, real_time=True, speed=2.0)

# Base classes
class RoboSuiteObject(Object):
    """Base class for RoboSuite objects."""
    # Physics properties available to all objects
    density: 1000
    friction: (1.0, 0.005, 0.0001)
    solref: (0.02, 1.0)
    solimp: (0.9, 0.95, 0.001, 0.5, 2.0)

# Basic objects with proper default sizes
class Cube(RoboSuiteObject):
    """Cubic object."""
    width: 0.05
    length: 0.05
    height: 0.05
    color: (0.8, 0.2, 0.2)  # Red default

class Ball(RoboSuiteObject):
    """Spherical object."""
    width: 0.05
    length: 0.05
    height: 0.05
    color: (0.2, 0.8, 0.2)  # Green default

class Cylinder(RoboSuiteObject):
    """Cylindrical object."""
    width: 0.05
    length: 0.05
    height: 0.1
    color: (0.2, 0.2, 0.8)  # Blue default

# Base Robot class
class Robot(RoboSuiteObject):
    """Base RoboSuite robot."""
    robot_type: "Panda"
    gripper_type: "default"
    controller_config: None
    initial_qpos: None
    base_type: "default"
    width: 0.2
    length: 0.2
    height: 0.5
    joint_positions: [] 

# Specific robot implementations
class PandaRobot(Robot):
    """Franka Emika Panda robot."""
    robot_type: "Panda"
    gripper_type: "PandaGripper"
    initial_qpos: [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04]
    height: 0.9

class SawyerRobot(Robot):
    """Rethink Sawyer robot."""
    robot_type: "Sawyer"
    gripper_type: "RethinkGripper"
    initial_qpos: [0, -0.785, 0, 1.571, 0, -0.785, 0]
    height: 1.0

# Arena classes
class EmptyArena(RoboSuiteObject):
    """Empty arena with just a floor."""
    width: 0
    length: 0
    height: 0

class TableArena(RoboSuiteObject):
    """Arena with a table in the center."""
    width: 0
    length: 0
    height: 0

class BinsArena(RoboSuiteObject):
    """Arena with bins for sorting tasks."""
    width: 0
    length: 0
    height: 0

class CustomArena(RoboSuiteObject):
    """Custom arena from XML."""
    xml_string: ""
    width: 0
    length: 0
    height: 0

# Table classes
class Table(RoboSuiteObject):
    """Standard table (part of TableArena)."""
    width: 1.2
    length: 0.8
    height: 0.8

class PositionableTable(RoboSuiteObject):
    """Table that can be positioned anywhere."""
    width: 1.0
    length: 0.8
    height: 0.8
    color: (0.9, 0.9, 0.9)  # Light gray

# XML-based objects
class XMLObject(RoboSuiteObject):
    """Object defined by XML file or string."""
    xml_path: None
    xml_string: None
    xml_size: None  # Optional size override [width, length, height]
    width: 0.1
    length: 0.1
    height: 0.1

class CustomXMLObject(XMLObject):
    """Convenience class for custom XML objects."""
    pass

# Actions
import scenic.core.dynamics as dynamics

class SetJointPositions(dynamics.Action):
    """Action to set robot joint positions."""
    def __init__(self, positions):
        self.positions = positions
    
    def applyTo(self, agent, sim):
        """Apply joint position action to robot."""
        pass