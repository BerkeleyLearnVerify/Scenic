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

# XML-based object support
class XMLObject(RoboSuiteObject):
    """Object defined via MuJoCo XML."""
    xml_path: None      # Path to XML file
    xml_string: None    # XML content as string
    
    # Material specification
    material: {}        # Dict with 'name', 'texture', 'attributes'

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

# Environment objects
class Arena(RoboSuiteObject):
    """Base class for RoboSuite arenas."""
    pass

class EmptyArena(Arena):
    """Empty arena with just floor."""
    pass

class TableArena(Arena):
    """Standard table arena."""
    table_height: 0.8
    table_width: 1.0
    table_length: 0.8

class BinsArena(Arena):
    """Two bins for pick-and-place tasks."""
    bin_size: 0.3
    bin_height: 0.1

class PegsArena(Arena):
    """Pegboard for insertion tasks."""
    board_size: 0.3
    peg_radius: 0.015

class WipeArena(Arena):
    """Table with markers for wiping tasks."""
    table_height: 0.8
    marker_size: 0.08

class CustomArena(Arena):
    """Custom arena defined via XML."""
    xml_string: None
    xml_path: None
    
    # Custom objects to add to empty arena
    objects: []

# Legacy table support
class Table(RoboSuiteObject):
    """Table object (creates TableArena)."""
    width: 1.0
    length: 0.8
    height: 0.8

class PositionableTable(RoboSuiteObject):
    """Table with full position control."""
    width: 1.0
    length: 0.8
    height: 0.8

class Bin(RoboSuiteObject):
    """Bin/container object."""
    width: 0.3
    length: 0.3
    height: 0.2

class Door(RoboSuiteObject):
    """Door object for manipulation."""
    width: 0.5
    length: 0.05
    height: 0.8

# XML-based custom objects
class CustomBox(XMLObject):
    """Example box with inline XML."""
    width: 0.1
    length: 0.1
    height: 0.1
    
    xml_string: '''
    <mujoco model="custom_box">
        <body name="box_main">
            <geom name="box_collision" type="box" size="0.05 0.05 0.05" 
                  group="0" rgba="0.5 0.5 0.5 1"/>
            <geom name="box_visual" type="box" size="0.05 0.05 0.05" 
                  group="1" rgba="1 0 0 1"/>
        </body>
    </mujoco>
    '''

class MeshObject(XMLObject):
    """Object loaded from mesh file."""
    pass  # Define xml_path in instance

# Deprecated - use XMLObject instead
class XMLBasedObject(XMLObject):
    """Deprecated: Use XMLObject instead."""
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