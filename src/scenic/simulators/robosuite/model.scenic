"""Scenic world model for RoboSuite with MJCF mesh extraction."""

from .simulator import RobosuiteSimulator, SetJointPositions, OSCPositionAction
from scenic.core.utils import repairMesh
import trimesh
import json
import tempfile
import os
from pathlib import Path
import numpy as np

# Global parameters
param env_config = {}
param controller_config = None
param camera_view = None
param render = True
param real_time = True
param speed = 1.0
param lite_physics = None
param scenic_file_dir = localPath(".")

# Load arena configuration
_arena_config_path = localPath("utils/arena_meshes/arena_config.json")
with open(_arena_config_path) as f:
    _arena_config = json.load(f)

_floor_surface_z = _arena_config['floor']['position'][2] + _arena_config['floor']['dimensions'][2] / 2
param floor_surface_z = _floor_surface_z

# Simulator
simulator RobosuiteSimulator(
    render=globalParameters.render,
    real_time=globalParameters.real_time,
    speed=globalParameters.speed,
    env_config=globalParameters.env_config,
    controller_config=globalParameters.controller_config,
    camera_view=globalParameters.camera_view,
    lite_physics=globalParameters.lite_physics
)

# Load robot dimensions
_robot_dims_path = localPath("utils/robot_meshes/dimensions.txt")
_robot_dimensions = {}
with open(_robot_dims_path) as f:
    for line in f:
        if ':' in line:
            robot, dims_str = line.strip().split(':')
            dims = eval(dims_str.strip())
            _robot_dimensions[robot] = dims

# Default values
DEFAULTS = {
    'object_size': 0.03,
    'density': 1000,
    'friction': (1.0, 0.005, 0.0001),
    'solref': (0.02, 1.0),
    'solimp': (0.9, 0.95, 0.001, 0.5, 2.0),
    'default_color': (0.5, 0.5, 0.5, 1.0),
    
    'table_height': 0.8,
    'table_width': 0.8,
    'table_length': 0.8,
    'table_thickness': 0.05,
    
    'control_gain': 3.0,
    'control_limit': 0.3,
    'position_tolerance': 0.02,
    'height_tolerance': 0.02,
    'gripper_open_steps': 20,
    'gripper_close_steps': 30,
    'max_control_steps': 100,
    'max_lift_steps': 200
}


def _mjcf_to_shape(mjcf_xml: str, scenic_file_path: str = None) -> Shape:
    """Convert MJCF XML to MeshShape via temp GLB file."""
    if not mjcf_xml:
        return BoxShape()
    
    import xml.etree.ElementTree as ET
    import uuid
    import shutil
    
    base_dir = scenic_file_path if scenic_file_path else os.getcwd()
    
    # Handle file path vs XML string
    xml_content = mjcf_xml
    if not mjcf_xml.strip().startswith('<'):
        if not os.path.isabs(mjcf_xml):
            xml_path = os.path.join(base_dir, mjcf_xml)
        else:
            xml_path = mjcf_xml
        
        if os.path.exists(xml_path):
            with open(xml_path, 'r') as f:
                xml_content = f.read()
            base_dir = os.path.dirname(xml_path)
        else:
            print(f"Warning: MJCF file not found: {xml_path}")
            return BoxShape()
    
    # Create temp directory
    temp_id = str(uuid.uuid4())[:8]
    temp_dir = Path(tempfile.gettempdir()) / f"scenic_mjcf_{temp_id}"
    temp_dir.mkdir(exist_ok=True)
    temp_glb = temp_dir / "converted.glb"
    
    try:
        root = ET.fromstring(xml_content)
        mesh_geom = root.find('.//geom[@mesh]')
        
        if mesh_geom is not None:
            # Handle mesh file
            mesh_name = mesh_geom.get('mesh')
            mesh_elem = root.find(f'.//asset/mesh[@name="{mesh_name}"]')
            mesh_file = mesh_elem.get('file')
            scale = np.array([float(s) for s in mesh_elem.get('scale', '1 1 1').split()])
            
            if not os.path.isabs(mesh_file):
                mesh_path = os.path.join(base_dir, mesh_file)
            else:
                mesh_path = mesh_file
            
            mesh = trimesh.load(mesh_path)
            if isinstance(mesh, trimesh.Scene):
                mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
            
            mesh.apply_scale(scale)
            
        else:
            # Handle primitives
            geoms = root.findall('.//geom[@type]')
            if not geoms:
                print("Warning: No mesh or primitive geom found")
                return BoxShape()
            
            meshes = []
            for geom in geoms:
                geom_type = geom.get('type')
                size = np.array([float(s) for s in geom.get('size', '0.1 0.1 0.1').split()])
                
                if geom_type == 'box':
                    geom_mesh = trimesh.creation.box(extents=size * 2)
                elif geom_type == 'sphere':
                    geom_mesh = trimesh.creation.icosphere(radius=size[0], subdivisions=2)
                elif geom_type == 'cylinder':
                    geom_mesh = trimesh.creation.cylinder(radius=size[0], height=size[1] * 2)
                elif geom_type == 'capsule':
                    geom_mesh = trimesh.creation.capsule(radius=size[0], height=size[1] * 2)
                else:
                    continue
                
                pos = np.array([float(p) for p in geom.get('pos', '0 0 0').split()])
                if np.any(pos != 0):
                    geom_mesh.apply_translation(pos)
                
                meshes.append(geom_mesh)
            
            if meshes:
                mesh = trimesh.util.concatenate(meshes)
            else:
                print("Warning: No valid primitives found")
                return BoxShape()
        
        # Export, load, repair sequence
        mesh.export(str(temp_glb))
        
        loaded_mesh = trimesh.load(str(temp_glb))
        if isinstance(loaded_mesh, trimesh.Scene):
            loaded_mesh = trimesh.util.concatenate(list(loaded_mesh.geometry.values()))
        
        repaired_mesh = repairMesh(loaded_mesh, pitch=0.02, verbose=False)
        
        repaired_glb = temp_dir / "repaired.glb"
        repaired_mesh.export(str(repaired_glb))
        
        shape = MeshShape.fromFile(str(repaired_glb))
        
        # Schedule cleanup
        def cleanup():
            if temp_dir.exists():
                shutil.rmtree(temp_dir, ignore_errors=True)
        
        import atexit
        atexit.register(cleanup)
        
        return shape
        
    except Exception as e:
        print(f"Error in _mjcf_to_shape: {e}")
        if temp_dir.exists():
            shutil.rmtree(temp_dir, ignore_errors=True)
        return BoxShape()


def _load_robot_mesh_shape(path):
    """Load mesh and create MeshShape with repair."""
    mesh = trimesh.load(path, force='mesh')
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)
    repaired_mesh = repairMesh(mesh, pitch=0.02, verbose=False)
    return MeshShape(repaired_mesh)

# Base classes
class RoboSuiteObject(Object):
    """Base class for all RoboSuite objects."""
    density: DEFAULTS['density']
    friction: DEFAULTS['friction']
    solref: DEFAULTS['solref']
    solimp: DEFAULTS['solimp']
    shape: BoxShape()
    allowCollisions: False
    requireVisible: False
    regionContainedIn: None

class ArenaFloor(RoboSuiteObject):
    """Floor component for Scenic's collision system."""
    shape: MeshShape.fromFile(localPath("utils/arena_meshes/floor.glb"))
    position: (0.0, 0.0, 0.0)
    width: _arena_config['floor']['dimensions'][0]
    length: _arena_config['floor']['dimensions'][1] 
    height: _arena_config['floor']['dimensions'][2]
    allowCollisions: True  
    requireVisible: False
    _isArenaComponent: True

class ArenaWalls(RoboSuiteObject):
    """Walls component for Scenic's collision system."""
    shape: MeshShape.fromFile(localPath("utils/arena_meshes/walls.glb"))
    width: _arena_config['walls']['dimensions'][0]
    length: _arena_config['walls']['dimensions'][1]
    height: _arena_config['walls']['dimensions'][2]
    allowCollisions: False
    requireVisible: False
    _isArenaComponent: True

# Create default arena components
arena_floor = new ArenaFloor
arena_walls = new ArenaWalls on (0.5, 0.0, 0.0)

class CustomArena(RoboSuiteObject):
    """Custom arena defined by complete MJCF XML."""
    isCustomArena: True
    arena_xml: ""

class Table(RoboSuiteObject):
    """Table in environment - creates MultiTableArena.
    
    WARNING: Table height is fixed and should not be modified.
    - Scenic uses full table mesh height (0.85m) for collision/visualization
    - Robosuite uses tabletop thickness (0.05m) for MultiTableArena
    Changing these values causes visual inconsistencies between simulators.
    """
    isTable: True
    shape: MeshShape.fromFile(localPath("utils/table_meshes/standard_table.glb"))
    width: DEFAULTS['table_width']
    length: DEFAULTS['table_length']
    # Height is fixed from mesh - DO NOT make configurable
    height: 0.85  # Full table height for Scenic collision
    position: (0, 0, 0.425)  # Center of table volume

class ManipulationObject(RoboSuiteObject):
    """Base class for manipulable objects."""
    color: DEFAULTS['default_color']

class CustomObject(ManipulationObject):
    """Custom object with automatic dimension extraction."""
    objectType: "MJCF"
    mjcf_xml: ""  
    shape: _mjcf_to_shape(self.mjcf_xml, globalParameters.scenic_file_dir) if self.mjcf_xml else BoxShape()

# Primitive objects
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

# Complex objects
class Milk(ManipulationObject):
    objectType: "Milk"

class Cereal(ManipulationObject):
    objectType: "Cereal"

class Can(ManipulationObject):
    objectType: "Can"

class Bread(ManipulationObject):
    objectType: "Bread"

class Bottle(ManipulationObject):
    objectType: "Bottle"

class Hammer(ManipulationObject):
    objectType: "Hammer"

class SquareNut(ManipulationObject):
    objectType: "SquareNut"

class RoundNut(ManipulationObject):
    objectType: "RoundNut"

# Robots
class Robot(RoboSuiteObject):
    """Base robot class."""
    robot_type: "Panda"
    shape: BoxShape()
    gripper_type: "default"
    controller_config: None
    initial_qpos: None
    base_type: "default"
    width: 0.4
    length: 0.4
    height: 1.0
    position: (0, 0, 0)
    
    joint_positions[dynamic]: []
    eef_pos[dynamic]: [0, 0, 0]
    gripper_state[dynamic]: [0, 0]

class Panda(Robot):
    robot_type: "Panda"
    shape: _load_robot_mesh_shape(localPath("utils/robot_meshes/Panda.glb"))
    width: _robot_dimensions.get("Panda", [0.4, 0.4, 1.0])[0]
    length: _robot_dimensions.get("Panda", [0.4, 0.4, 1.0])[1]
    height: _robot_dimensions.get("Panda", [0.4, 0.4, 1.0])[2]
    gripper_type: "PandaGripper"

class UR5e(Robot):
    robot_type: "UR5e"
    shape: _load_robot_mesh_shape(localPath("utils/robot_meshes/UR5e.glb"))
    width: _robot_dimensions.get("UR5e", [0.4, 0.4, 1.0])[0]
    length: _robot_dimensions.get("UR5e", [0.4, 0.4, 1.0])[1]
    height: _robot_dimensions.get("UR5e", [0.4, 0.4, 1.0])[2]
    gripper_type: "Robotiq85Gripper"

class Jaco(Robot):
    robot_type: "Jaco"
    shape: _load_robot_mesh_shape(localPath("utils/robot_meshes/Jaco.glb"))
    width: _robot_dimensions.get("Jaco", [0.4, 0.4, 1.0])[0]
    length: _robot_dimensions.get("Jaco", [0.4, 0.4, 1.0])[1]
    height: _robot_dimensions.get("Jaco", [0.4, 0.4, 1.0])[2]
    gripper_type: "JacoThreeFingerGripper"

class IIWA(Robot):
    robot_type: "IIWA"
    shape: _load_robot_mesh_shape(localPath("utils/robot_meshes/IIWA.glb"))
    width: _robot_dimensions.get("IIWA", [0.4, 0.4, 1.0])[0]
    length: _robot_dimensions.get("IIWA", [0.4, 0.4, 1.0])[1]
    height: _robot_dimensions.get("IIWA", [0.4, 0.4, 1.0])[2]
    gripper_type: "Robotiq140Gripper"

# Behaviors
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