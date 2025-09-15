# src/scenic/simulators/robosuite/simulator.py
"""RoboSuite Simulator interface for Scenic - Clean Architecture."""

from typing import Dict, List, Any, Optional, Union
import numpy as np
import mujoco
import os
import tempfile
import shutil
import xml.etree.ElementTree as ET

try:
    import robosuite as suite
    from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
    from robosuite.models.arenas import Arena, EmptyArena, MultiTableArena
    from robosuite.models.tasks import ManipulationTask
    from robosuite.models.objects import (
        MujocoXMLObject,
        BallObject, BoxObject, CylinderObject, CapsuleObject,
        MilkObject, CerealObject, CanObject, BreadObject,
        HammerObject, SquareNutObject, RoundNutObject, BottleObject
    )
    from robosuite.utils.observables import Observable, sensor
    from robosuite.utils.mjcf_utils import xml_path_completion
except ImportError as e:
    suite = None
    _import_error = e

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector
from scenic.core.dynamics import Action

# Constants
DEFAULT_PHYSICS_TIMESTEP = 0.002
DEFAULT_TIMESTEP = 0.01
DEFAULT_ACTION_DIM = 7
PHYSICS_SETTLE_STEPS = 10

# Camera view mapping
CAMERA_VIEWS = {
    "frontview": 0,
    "birdview": 1,
    "agentview": 2,
    "sideview": 3,
    "robot0_robotview": 4,
    "robot0_eye_in_hand": 5
}


class CustomXMLArena(Arena):
    """Arena created from custom XML string or file."""
    
    def __init__(self, xml_string, scenic_file_path=None):
        # Initialize with empty arena base
        super().__init__(xml_path_completion("arenas/empty_arena.xml"))
        
        # Check if xml_string is a file path
        if not xml_string.strip().startswith('<'):
            # It's a file path - resolve it
            base_dir = os.path.dirname(scenic_file_path) if scenic_file_path else os.getcwd()
            if not os.path.isabs(xml_string):
                xml_path = os.path.join(base_dir, xml_string)
            else:
                xml_path = xml_string
            
            if os.path.exists(xml_path):
                with open(xml_path, 'r') as f:
                    xml_string = f.read()
            else:
                raise FileNotFoundError(f"Arena XML file not found: {xml_path}")
        
        # Parse custom XML
        custom_tree = ET.fromstring(xml_string)
        
        # Get worldbody elements from custom XML
        custom_worldbody = custom_tree.find(".//worldbody")
        if custom_worldbody is None:
            raise ValueError("Custom XML must have a worldbody element")
        
        # Clear existing worldbody content but keep the worldbody element
        for child in list(self.worldbody):
            self.worldbody.remove(child)
        
        # Add all elements from custom worldbody directly
        for element in custom_worldbody:
            self.worldbody.append(element)
        
        # Merge assets
        custom_asset = custom_tree.find(".//asset")
        if custom_asset is not None and self.asset is not None:
            for child in custom_asset:
                self.asset.append(child)
        
        # Set floor reference
        self.floor = self.worldbody.find(".//geom[@name='floor']")
        if self.floor is None:
            for geom in self.worldbody.findall(".//geom"):
                if geom.get("type") == "plane":
                    self.floor = geom
                    break
        
        # Set table references if they exist
        self.table_body = self.worldbody.find(".//body[@name='custom_table']")
        self.table_top = self.worldbody.find(".//geom[@name='table_top']")


class CustomMeshObject(MujocoXMLObject):
    """
    Custom manipulable object from XML string.
    Automatically ensures free joint and collision geometry.
    Automatically resolves mesh/texture file paths relative to the Scenic file location.
    """
    
    # Class variable to track all free joint names to avoid conflicts
    _existing_joint_names = set()
    
    def __init__(self, name, xml_string, scenic_file_path=None):
        self.object_name = name
        
        # Create a temporary directory for this object
        self.temp_dir = tempfile.mkdtemp()
        
        # Get base directory for relative paths
        base_dir = scenic_file_path if scenic_file_path else os.getcwd()
        
        # Check if xml_string is a file path
        if not xml_string.strip().startswith('<'):
            # It's a file path - resolve it
            if not os.path.isabs(xml_string):
                xml_path = os.path.join(base_dir, xml_string)
            else:
                xml_path = xml_string
            
            if os.path.exists(xml_path):
                with open(xml_path, 'r') as f:
                    xml_string = f.read()
                # Update base_dir to the XML file's directory for relative asset paths
                base_dir = os.path.dirname(xml_path)
            else:
                raise FileNotFoundError(f"Object XML file not found: {xml_path}")
        
        # Parse XML
        tree = ET.fromstring(xml_string)
        
        # Find and copy all mesh and texture files referenced in XML
        self._process_asset_files(tree, base_dir)
        
        # Process the XML to ensure joint and collision
        tree = self._process_xml_tree(tree, name)
        
        # Convert back to string
        fixed_xml = ET.tostring(tree, encoding='unicode')
        
        # Write XML to temp file
        temp_xml_path = os.path.join(self.temp_dir, f"{name}.xml")
        with open(temp_xml_path, 'w') as f:
            f.write(fixed_xml)
        
        try:
            super().__init__(
                temp_xml_path,
                name=name,
                joints=None,  # Joint is in the XML
                obj_type="all",
                duplicate_collision_geoms=False
            )
        finally:
            pass  # Keep temp_dir for MuJoCo to access files
    
    def _process_asset_files(self, tree, base_dir):
        """Find and copy all asset files referenced in the XML."""
        
        # Process mesh files
        for mesh in tree.findall(".//mesh"):
            file_attr = mesh.get("file")
            if file_attr:
                # Resolve the path relative to base_dir
                if not os.path.isabs(file_attr):
                    full_path = os.path.join(base_dir, file_attr)
                else:
                    full_path = file_attr
                
                if os.path.exists(full_path):
                    # Copy to temp directory
                    filename = os.path.basename(full_path)
                    temp_path = os.path.join(self.temp_dir, filename)
                    shutil.copy2(full_path, temp_path)
                    
                    # Update XML to use just the filename
                    mesh.set("file", filename)
                    
                    # Auto-copy MTL file for OBJ meshes
                    if full_path.endswith('.obj'):
                        mtl_path = full_path.replace('.obj', '.mtl')
                        if os.path.exists(mtl_path):
                            mtl_filename = os.path.basename(mtl_path)
                            temp_mtl_path = os.path.join(self.temp_dir, mtl_filename)
                            shutil.copy2(mtl_path, temp_mtl_path)
                else:
                    print(f"Warning: Mesh file not found: {full_path}")
        
        # Process texture files
        for texture in tree.findall(".//texture"):
            file_attr = texture.get("file")
            if file_attr and not texture.get("builtin"):  # Skip builtin textures
                # Resolve the path relative to base_dir
                if not os.path.isabs(file_attr):
                    full_path = os.path.join(base_dir, file_attr)
                else:
                    full_path = file_attr
                
                if os.path.exists(full_path):
                    filename = os.path.basename(full_path)
                    
                    # Convert JPG to PNG if needed
                    if filename.lower().endswith(('.jpg', '.jpeg')):
                        try:
                            from PIL import Image
                            img = Image.open(full_path)
                            filename = filename.rsplit('.', 1)[0] + '.png'
                            temp_path = os.path.join(self.temp_dir, filename)
                            img.save(temp_path)
                        except ImportError:
                            temp_path = os.path.join(self.temp_dir, filename)
                            shutil.copy2(full_path, temp_path)
                    else:
                        temp_path = os.path.join(self.temp_dir, filename)
                        shutil.copy2(full_path, temp_path)
                    
                    # Update XML to use just the filename
                    texture.set("file", filename)
                else:
                    print(f"Warning: Texture file not found: {full_path}")
    
    def _process_xml_tree(self, tree, name):
        """Process XML tree to ensure joint and collision geometry."""
        
        # Find the object body (usually named "object")
        object_body = tree.find(".//body[@name='object']")
        if object_body is None:
            # Try to find any body that's a child of worldbody
            worldbody = tree.find(".//worldbody")
            if worldbody is not None:
                for body in worldbody.findall("body"):
                    inner_body = body.find("body[@name='object']")
                    if inner_body is not None:
                        object_body = inner_body
                        break
        
        if object_body is None:
            print(f"Warning: Could not find object body for {name}")
            return tree
        
        # ALWAYS ensure free joint for manipulable objects
        self._ensure_free_joint(object_body, name)
        
        # ALWAYS ensure collision geometry exists
        self._ensure_collision_geometry(object_body, name)
        
        return tree
    
    def _ensure_free_joint(self, object_body, name):
        """Ensure object has a free joint for physics."""
        has_free_joint = False
        existing_joint_name = None
        
        for joint in object_body.findall("joint"):
            if joint.get("type") == "free":
                has_free_joint = True
                existing_joint_name = joint.get("name")
                if existing_joint_name:
                    self._existing_joint_names.add(existing_joint_name)
                    self.joint_name = existing_joint_name
                break
        
        if not has_free_joint:
            # Generate unique joint name
            joint_counter = 0
            while True:
                joint_name = f"{name}_joint_{joint_counter}"
                if joint_name not in self._existing_joint_names:
                    break
                joint_counter += 1
            
            # Create and add joint element
            joint = ET.Element("joint")
            joint.set("name", joint_name)
            joint.set("type", "free")
            joint.set("damping", "0.0005")
            
            # Insert joint as first child of body
            object_body.insert(0, joint)
            self._existing_joint_names.add(joint_name)
            self.joint_name = joint_name
    
    def _ensure_collision_geometry(self, object_body, name):
        """Always ensure proper collision geometry exists."""
        has_proper_collision = False
        mesh_collision_geom = None
        visual_geom = None
        
        # Check what's already there
        for geom in object_body.findall("geom"):
            geom_group = geom.get("group", "0")
            geom_type = geom.get("type", "box")
            
            if geom_group == "0":
                # Found collision geometry
                if geom_type == "mesh":
                    # Mesh collision is problematic - needs to be fixed
                    mesh_collision_geom = geom
                    print(f"Found mesh collision geometry for {name} - will convert to box")
                else:
                    # Proper primitive collision geometry
                    has_proper_collision = True
                    break
            elif geom_group == "1":
                visual_geom = geom
        
        # Handle mesh collision geometry - convert to visual + box collision
        if mesh_collision_geom is not None and not has_proper_collision:
            # First, convert the mesh geom to visual (group 1)
            mesh_collision_geom.set("group", "1")
            mesh_collision_geom.set("contype", "0")
            mesh_collision_geom.set("conaffinity", "0")
            
            # Create a box collision geom
            collision_geom = ET.Element("geom")
            collision_geom.set("name", "collision_auto")
            collision_geom.set("type", "box")
            collision_geom.set("size", "0.04 0.04 0.04")
            
            # Copy position from mesh if available
            pos = mesh_collision_geom.get("pos", "0 0 0")
            collision_geom.set("pos", pos)
            
            # Set collision properties
            collision_geom.set("group", "0")
            collision_geom.set("rgba", "0 0 0 0")  # Invisible
            
            # Copy collision properties from original mesh if they exist
            for prop in ["solimp", "solref", "density", "friction"]:
                value = mesh_collision_geom.get(prop)
                if value:
                    collision_geom.set(prop, value)
            
            # If no collision properties were set, use defaults
            if not collision_geom.get("solimp"):
                collision_geom.set("solimp", "0.998 0.998 0.001")
            if not collision_geom.get("solref"):
                collision_geom.set("solref", "0.001 1")
            if not collision_geom.get("density"):
                collision_geom.set("density", "100")
            if not collision_geom.get("friction"):
                collision_geom.set("friction", "0.95 0.3 0.1")
            
            # Add collision geom after the converted visual geom
            index = list(object_body).index(mesh_collision_geom) + 1
            object_body.insert(index, collision_geom)
            
            print(f"Converted mesh collision to visual + box collision for {name}")
            return
        
        # If no collision geometry at all, create it
        if not has_proper_collision:
            if visual_geom is not None:
                # Create collision based on visual
                collision_geom = ET.Element("geom")
                collision_geom.set("name", "collision_auto")
                
                # Get type and position from visual
                geom_type = visual_geom.get("type", "box")
                pos = visual_geom.get("pos", "0 0 0")
                
                # For mesh, use box approximation
                if geom_type == "mesh":
                    collision_geom.set("type", "box")
                    collision_geom.set("size", "0.04 0.04 0.04")
                else:
                    # For primitives, copy the shape
                    collision_geom.set("type", geom_type)
                    size = visual_geom.get("size")
                    if size:
                        collision_geom.set("size", size)
                
                collision_geom.set("pos", pos)
                
            else:
                # No visual geom either - create default box collision
                collision_geom = ET.Element("geom")
                collision_geom.set("name", "collision_default")
                collision_geom.set("type", "box")
                collision_geom.set("size", "0.04 0.04 0.04")
                collision_geom.set("pos", "0 0 0")
            
            # Set collision properties
            collision_geom.set("group", "0")
            collision_geom.set("rgba", "0 0 0 0")  # Invisible
            collision_geom.set("solimp", "0.998 0.998 0.001")
            collision_geom.set("solref", "0.001 1")
            collision_geom.set("density", "100")
            collision_geom.set("friction", "0.95 0.3 0.1")
            
            # Add collision geom
            if visual_geom is not None:
                # Add after visual geom
                index = list(object_body).index(visual_geom) + 1
                object_body.insert(index, collision_geom)
            else:
                # Just append to body
                object_body.append(collision_geom)
            
            print(f"Added collision geometry to {name}")
    
    def __del__(self):
        """Clean up temp directory when object is destroyed."""
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

def _extract_cylinder_size(config: Dict) -> List[float]:
    """Extract [radius, height] for cylinder-like objects."""
    if 'radius' in config and 'height' in config:
        return [config['radius'], config['height']]
    
    size = config.get('size', [0.02, 0.02, 0.04])
    if len(size) == 3:  # Convert from [width, length, height]
        radius = (size[0] + size[1]) / 4  
        return [radius, size[2]]
    elif len(size) == 2:
        return size
    return [0.02, 0.04]


# Object type mapping for built-in RoboSuite objects
OBJECT_FACTORIES = {
    # Primitive objects with customizable size
    'Ball': lambda cfg: BallObject(
        name=cfg['name'],
        size=[cfg.get('radius', 0.02)],
        rgba=cfg.get('color', [1, 0, 0, 1])
    ),
    'Box': lambda cfg: BoxObject(
        name=cfg['name'],
        size=cfg.get('size', [0.025, 0.025, 0.025]),
        rgba=cfg.get('color', [1, 0, 0, 1])
    ),
    'Cylinder': lambda cfg: CylinderObject(
        name=cfg['name'],
        size=_extract_cylinder_size(cfg),
        rgba=cfg.get('color', [1, 0, 0, 1])
    ),
    'Capsule': lambda cfg: CapsuleObject(
        name=cfg['name'],
        size=_extract_cylinder_size(cfg),
        rgba=cfg.get('color', [1, 0, 0, 1])
    ),
    # Complex objects with fixed sizes
    'Milk': lambda cfg: MilkObject(name=cfg['name']),
    'Cereal': lambda cfg: CerealObject(name=cfg['name']),
    'Can': lambda cfg: CanObject(name=cfg['name']),
    'Bread': lambda cfg: BreadObject(name=cfg['name']),
    'Hammer': lambda cfg: HammerObject(name=cfg['name']),
    'SquareNut': lambda cfg: SquareNutObject(name=cfg['name']),
    'RoundNut': lambda cfg: RoundNutObject(name=cfg['name']),
    'Bottle': lambda cfg: BottleObject(name=cfg['name'])
}


class ScenicManipulationEnv(ManipulationEnv):
    """Scenic-driven manipulation environment."""
    
    def __init__(self, scenic_config: Dict, **kwargs):
        self.scenic_config = scenic_config
        self.scenic_objects = scenic_config.get('objects', [])
        self.scenic_tables = scenic_config.get('tables', [])
        self.scenic_custom_arena = scenic_config.get('custom_arena', None)
        super().__init__(**kwargs)
    
    def _load_model(self):
        """Load models and create arena."""
        super()._load_model()
        
        self._position_robots()
        mujoco_arena = self._create_arena()
        mujoco_arena.set_origin([0, 0, 0])
        
        self.mujoco_objects = self._create_objects()
        
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.mujoco_objects
        )
    
    def _position_robots(self):
        """Position robots based on scenic configuration."""
        for i, robot_config in enumerate(self.scenic_config.get('robots', [])):
            if i < len(self.robots):
                pos = robot_config.get('position', [0, 0, 0])
                self.robots[i].robot_model.set_base_xpos(pos)
    
# Updated _create_arena method in ScenicManipulationEnv class

    def _create_arena(self):
        """Create arena based on configuration."""
        # Custom arena takes priority
        if self.scenic_custom_arena:
            xml_string = self.scenic_custom_arena.get('xml', '')
            scenic_file_path = self.scenic_custom_arena.get('scenic_file_path', None)
            print(f"DEBUG: Creating custom arena with scenic_file_path: {scenic_file_path}")
            return CustomXMLArena(xml_string, scenic_file_path)
        
        # Tables create MultiTableArena
        if self.scenic_tables:
            return MultiTableArena(
                table_offsets=[t.get('position', [0, 0, 0.8]) for t in self.scenic_tables],
                table_full_sizes=[t.get('size', (0.8, 0.8, 0.05)) for t in self.scenic_tables],
                has_legs=[True] * len(self.scenic_tables)
            )
        
        # Default to empty arena
        return EmptyArena()
    

    def _create_objects(self) -> List:
        """Create Robosuite objects from Scenic configuration."""
        mujoco_objects = []
        
        for i, obj_config in enumerate(self.scenic_objects):
            obj_type = obj_config['type']
            obj_name = obj_config.get('name', f'unnamed_{i}')
            
            if obj_type == 'MJCF':
                # Custom MJCF object - joint and collision always ensured
                scenic_file_path = obj_config.get('scenic_file_path', None)
                
                mj_obj = CustomMeshObject(
                    name=obj_name,
                    xml_string=obj_config.get('mjcf_xml', ''),
                    scenic_file_path=scenic_file_path
                    # No auto_add_joint or auto_add_collision - always automatic
                )
            else:
                # Built-in RoboSuite object
                factory = OBJECT_FACTORIES.get(obj_type, OBJECT_FACTORIES['Box'])
                mj_obj = factory(obj_config)
            
            mujoco_objects.append(mj_obj)
        
        return mujoco_objects

    def _setup_references(self):
        """Setup references to simulation objects."""
        super()._setup_references()
        self.obj_body_ids = {}
        
        for obj in self.mujoco_objects:
            try:
                # Try to find the object's body
                if hasattr(obj, 'root_body'):
                    body_name = obj.root_body
                else:
                    body_name = obj.name
                
                body_id = self.sim.model.body_name2id(body_name)
                self.obj_body_ids[obj.name] = body_id
            except:
                # Try with different naming patterns
                for suffix in ['_main', '_body', '']:
                    try:
                        body_name = f"{obj.name}{suffix}"
                        body_id = self.sim.model.body_name2id(body_name)
                        self.obj_body_ids[obj.name] = body_id
                        break
                    except:
                        continue
    
    def _setup_observables(self) -> Dict:
        """Setup observables for objects."""
        observables = super()._setup_observables()
        
        for obj in self.mujoco_objects:
            if obj.name in self.obj_body_ids:
                @sensor(modality="object")
                def obj_pos(obs_cache, name=obj.name):
                    return np.array(self.sim.data.body_xpos[self.obj_body_ids[name]])
                
                obj_pos.__name__ = f"{obj.name}_pos"
                observables[obj_pos.__name__] = Observable(
                    name=obj_pos.__name__,
                    sensor=obj_pos,
                    sampling_rate=self.control_freq
                )
        
        return observables
    
    def _reset_internal(self):
        """Reset environment internals."""
        super()._reset_internal()
        
        # Set positions for all objects
        for obj_config, mj_obj in zip(self.scenic_objects, self.mujoco_objects):
            if 'position' in obj_config:
                pos = obj_config['position']
                quat = obj_config.get('quaternion', [1, 0, 0, 0])
                
                # Find the joint name - RoboSuite prefixes object name to joints
                joint_name = None
                
                # Try multiple naming patterns
                possible_names = []
                
                if hasattr(mj_obj, 'joint_name'):
                    # For custom objects with stored joint name
                    base_joint = mj_obj.joint_name
                    possible_names.append(f"{mj_obj.name}_{base_joint}")
                    possible_names.append(base_joint)
                
                if hasattr(mj_obj, 'joints') and mj_obj.joints:
                    # For built-in objects with joints list
                    possible_names.extend(mj_obj.joints)
                
                # Also try standard patterns
                possible_names.append(f"{mj_obj.name}_joint0")
                possible_names.append(f"{mj_obj.name}_object_joint")
                
                # Find first matching joint
                for name in possible_names:
                    if name in self.sim.model.joint_names:
                        joint_name = name
                        break
                
                if joint_name:
                    self.sim.data.set_joint_qpos(
                        joint_name,
                        np.concatenate([np.array(pos), np.array(quat)])
                    )
                else:
                    print(f"Warning: Could not find joint for {mj_obj.name}")
                    print(f"  Available joints: {[j for j in self.sim.model.joint_names if mj_obj.name.lower() in j.lower()]}")
        
        # Step simulation to apply changes
        self.sim.forward()
        self.sim.step()
    
    def reward(self, action=None) -> float:
        """Compute reward."""
        return 0.0
    
    def _check_success(self) -> bool:
        """Check task success."""
        return False


class RobosuiteSimulator(Simulator):
    """Simulator for RoboSuite custom environments."""
    
    def __init__(self, render: bool = True, real_time: bool = True, speed: float = 1.0,
                 env_config: Optional[Dict] = None, controller_config: Optional[Dict] = None,
                 camera_view: Optional[str] = None, lite_physics: Optional[bool] = None):
        super().__init__()
        if suite is None:
            raise RuntimeError(f"Unable to import RoboSuite: {_import_error}")
        
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.env_config = env_config or {}
        self.controller_config = controller_config
        self.camera_view = camera_view
        self.lite_physics = lite_physics
    
    def createSimulation(self, scene, **kwargs):
        """Create a simulation instance."""
        return RobosuiteSimulation(
            scene, self.render, self.real_time, self.speed,
            self.env_config, self.controller_config,
            self.camera_view, self.lite_physics, **kwargs
        )


class RobosuiteSimulation(Simulation):
    """Simulation for RoboSuite custom environments."""
    
    def __init__(self, scene, render: bool, real_time: bool, speed: float,
                 env_config: Optional[Dict], controller_config: Optional[Dict],
                 camera_view: Optional[str], lite_physics: Optional[bool], **kwargs):
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.env_config = env_config or {}
        self.controller_config = controller_config
        self.camera_view = camera_view
        self.lite_physics = lite_physics
        
        # Environment state
        self.robosuite_env = None
        self.model = None
        self.data = None
        
        # Object tracking
        self.body_id_map = {}
        self.object_name_map = {}
        self.robots = []
        self.prev_positions = {}
        self._current_obs = None
        
        # Action handling
        self.pending_robot_action = None
        self.action_dim = DEFAULT_ACTION_DIM
        self.controller_type = None
        
        # Timing
        self.timestep = kwargs.get('timestep') or DEFAULT_TIMESTEP
        self.physics_timestep = kwargs.get('physics_timestep') or DEFAULT_PHYSICS_TIMESTEP
        self.physics_steps = int(self.timestep / self.physics_timestep)
        self.agents = []
        
        super().__init__(scene, **kwargs)
    
    def setup(self):
        """Initialize the RoboSuite environment."""
        super().setup()
        
        # Extract configuration from Scenic scene
        scenic_config = self._extract_scenic_config()
        
        # Check for robots - at least one required
        if not scenic_config['robots']:
            raise ValueError("At least one robot is required in the scene")
        
        # Setup environment
        robot_arg = self._get_robot_arg(scenic_config['robots'])
        
        env_kwargs = {
            'scenic_config': scenic_config,
            'robots': robot_arg,
            'has_renderer': self.render,
            'render_camera': self.camera_view,
            'camera_names': ["frontview", "robot0_eye_in_hand"],
            'controller_configs': self.controller_config,
            **self.env_config
        }
        
        if self.lite_physics is not None:
            env_kwargs['lite_physics'] = self.lite_physics
        
        self.robosuite_env = ScenicManipulationEnv(**env_kwargs)
        self._current_obs = self.robosuite_env.reset()
        self._detect_controller_type(scenic_config['robots'])
        
        # Finalize setup
        self._finalize_setup()
    
    def _finalize_setup(self):
        """Common setup after environment creation."""
        self.model = self.robosuite_env.sim.model._model
        self.data = self.robosuite_env.sim.data._data
        
        if self.render and self.camera_view is not None:
            self._set_camera_view()
        
        self._setup_body_mapping()
        
        # Identify agents
        self.agents = [obj for obj in self.objects 
                      if hasattr(obj, 'behavior') and obj.behavior]
    
    def _set_camera_view(self):
        """Set the camera view if specified."""
        camera_id = (CAMERA_VIEWS.get(self.camera_view.lower(), 0) 
                    if isinstance(self.camera_view, str) 
                    else self.camera_view)
        if self.robosuite_env.viewer:
            self.robosuite_env.viewer.set_camera(camera_id=camera_id)
    


    # Updated _extract_scenic_config method in RobosuiteSimulation class

    def _extract_scenic_config(self) -> Dict:
        """Extract configuration from Scenic scene."""
        config = {'robots': [], 'tables': [], 'objects': [], 'custom_arena': None}
        
        # Get the Scenic file directory - try multiple approaches
        scenic_file_path = None
        
        # Approach 1: Check if user explicitly set scenic_file_dir param
        if 'scenic_file_dir' in self.scene.params:
            path_obj = self.scene.params.get('scenic_file_dir')
            if path_obj:
                scenic_file_path = str(path_obj)
                print(f"Found scenic_file_dir from params: {scenic_file_path}")
        
        # Approach 2: Try to get from command line arguments
        if not scenic_file_path:
            import sys
            for arg in sys.argv:
                if arg.endswith('.scenic'):
                    scenic_file_abspath = os.path.abspath(arg)
                    if os.path.exists(scenic_file_abspath):
                        scenic_file_path = os.path.dirname(scenic_file_abspath)
                        print(f"Found scenic file path from sys.argv: {scenic_file_path}")
                        break
        
        # Final fallback: Use current working directory with a warning
        if not scenic_file_path:
            scenic_file_path = os.getcwd()
            print(f"WARNING: Could not determine Scenic file directory automatically!")
            print(f"  Using current working directory: {scenic_file_path}")
            print(f"  For correct relative paths, add this to your .scenic file:")
            print(f"    param scenic_file_dir = localPath(\".\")")
        
        print(f"DEBUG: Final scenic file directory: {scenic_file_path}")
        
        # Process all objects in the scene
        for obj in self.objects:
            if hasattr(obj, 'robot_type'):
                self._add_robot_config(config['robots'], obj)
            elif hasattr(obj, 'isCustomArena') and obj.isCustomArena:
                config['custom_arena'] = {
                    'xml': getattr(obj, 'arena_xml', ''),
                    'scenic_file_path': scenic_file_path
                }
            elif hasattr(obj, 'isTable') and obj.isTable:
                self._add_table_config(config['tables'], obj)
            elif hasattr(obj, 'objectType'):
                self._add_object_config(config['objects'], obj, scenic_file_path)
        
        return config
    
    
    def _add_robot_config(self, robots: List, obj):
        """Add robot configuration."""
        robots.append({
            'type': obj.robot_type,
            'position': [obj.position.x, obj.position.y, obj.position.z]
        })
        self.robots.append(obj)
    
    def _add_table_config(self, tables: List, obj):
        """Add table configuration."""
        tables.append({
            'position': [obj.position.x, obj.position.y, obj.position.z],
            'size': (obj.width, obj.length, obj.height)
        })
    
    def _add_object_config(self, objects: List, obj, scenic_file_path):
        """Add object configuration."""
        scenic_name = getattr(obj, 'name', None)
        
        if obj.objectType == 'MJCF':
            obj_name = f"{scenic_name}_{len(objects)}" if scenic_name else f"mjcf_object_{len(objects)}"
        else:
            base_name = scenic_name if scenic_name else obj.objectType
            obj_name = f"{base_name}_{len(objects)}"
        
        config = {
            'type': obj.objectType,
            'name': obj_name,
            'position': [obj.position.x, obj.position.y, obj.position.z],
            'color': getattr(obj, 'color', [1, 0, 0, 1])
        }
        
        self.object_name_map[obj] = obj_name
        
        # Handle MJCF objects
        if obj.objectType == 'MJCF':
            config['mjcf_xml'] = getattr(obj, 'mjcf_xml', '')
            config['scenic_file_path'] = scenic_file_path
            # No auto_add_joint or auto_add_collision - always automatic
        elif obj.objectType == 'Ball' and hasattr(obj, 'radius'):
            config['radius'] = obj.radius
        elif obj.objectType in ['Box', 'Cylinder', 'Capsule'] and hasattr(obj, 'width'):
            config['size'] = [obj.width, obj.length, obj.height]
        
        objects.append(config)


    def _get_robot_arg(self, robots: List) -> Union[str, List[str]]:
        """Get robot argument for environment creation."""
        if not robots:
            raise ValueError("At least one robot is required")
        return ([r['type'] for r in robots] if len(robots) > 1 
                else robots[0]['type'])
    
    def _detect_controller_type(self, robot_configs: List):
        """Detect controller type from first robot."""
        if robot_configs and self.robosuite_env.robots:
            first_robot = self.robosuite_env.robots[0]
            if hasattr(first_robot, 'controller'):
                self.controller_type = type(first_robot.controller).__name__
                if hasattr(first_robot.controller, 'control_dim'):
                    self.action_dim = first_robot.controller.control_dim
    
    def _setup_body_mapping(self):
        """Map environment objects to MuJoCo body IDs."""
        for obj in self.objects:
            if obj in self.object_name_map:
                self._map_object_body(obj)
    
    def _map_object_body(self, obj):
        """Map single object to body ID."""
        obj_name = self.object_name_map.get(obj)
        if not obj_name:
            return
        
        # Try to find the body
        for suffix in ['', '_main', '_body']:
            body_name = f"{obj_name}{suffix}"
            try:
                body_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
                )
                if body_id != -1:
                    self.body_id_map[obj_name] = body_id
                    self.prev_positions[obj_name] = self.data.xpos[body_id].copy()
                    break
            except:
                continue
    
    def createObjectInSimulator(self, obj):
        """Required by Scenic's Simulator interface."""
        # Objects are created during setup in ScenicManipulationEnv
        pass
    
    def executeActions(self, allActions: Dict[Any, List]) -> None:
        """Execute actions by calling their applyTo methods."""
        super().executeActions(allActions)
        
        self.pending_robot_action = None
        
        for agent in self.agents:
            if agent in allActions and allActions[agent]:
                for action in allActions[agent]:
                    if action and hasattr(action, 'applyTo'):
                        action.applyTo(agent, self)
                        break
    
    def step(self):
        """Step the simulation forward one timestep."""
        if hasattr(self, '_done') and self._done:
            return
        
        # Store previous positions
        for name, body_id in self.body_id_map.items():
            self.prev_positions[name] = self.data.xpos[body_id].copy()
        
        action = (self.pending_robot_action if self.pending_robot_action is not None 
                 else np.zeros(self.action_dim))
        
        # Step simulation
        for _ in range(self.physics_steps):
            obs, reward, done, info = self.robosuite_env.step(action)
            self._current_obs = obs
            self._done = done
            if self.render:
                self.robosuite_env.render()
            if done:
                break
        
        self.pending_robot_action = None
    
    def getProperties(self, obj, properties: List[str]) -> Dict[str, Any]:
        """Get current property values for an object."""
        values = {}
        robot_idx = self.robots.index(obj) if obj in self.robots else None
        
        for prop in properties:
            if prop == 'position':
                values[prop] = self._get_object_position(obj)
            elif prop == 'joint_positions' and robot_idx is not None:
                values[prop] = self._get_robot_joints(robot_idx)
            elif prop == 'eef_pos' and robot_idx is not None:
                values[prop] = self._get_eef_position(robot_idx)
            elif prop == 'gripper_state' and robot_idx is not None:
                values[prop] = self._get_gripper_state(robot_idx)
            else:
                values[prop] = getattr(obj, prop, None)
        
        return values
    
    def _get_object_position(self, obj) -> Vector:
        """Get object position."""
        obj_name = self.object_name_map.get(obj)
        if obj_name and self._current_obs:
            obs_key = f"{obj_name}_pos"
            if obs_key in self._current_obs:
                pos = self._current_obs[obs_key]
                return Vector(pos[0], pos[1], pos[2])
            elif obj_name in self.body_id_map:
                body_id = self.body_id_map[obj_name]
                pos = self.data.xpos[body_id]
                return Vector(pos[0], pos[1], pos[2])
        return obj.position
    
    def _get_robot_joints(self, robot_idx: int) -> List:
        """Get robot joint positions."""
        if robot_idx < len(self.robosuite_env.robots):
            return list(self.robosuite_env.robots[robot_idx]._joint_positions)
        return []
    
    def _get_eef_position(self, robot_idx: int) -> List:
        """Get end-effector position."""
        if self._current_obs:
            return list(self._current_obs.get(f'robot{robot_idx}_eef_pos', [0, 0, 0]))
        return [0, 0, 0]
    
    def _get_gripper_state(self, robot_idx: int) -> List:
        """Get gripper state."""
        if self._current_obs:
            return list(self._current_obs.get(f'robot{robot_idx}_gripper_qpos', [0, 0]))
        return [0, 0]
    
    def getCurrentObservation(self) -> Optional[Dict]:
        """Get current observation dictionary."""
        return self._current_obs
    
    def checkSuccess(self) -> bool:
        """Check if task is successfully completed."""
        if hasattr(self.robosuite_env, '_check_success'):
            return self.robosuite_env._check_success()
        return False
    
    def destroy(self):
        """Clean up simulation resources."""
        if self.robosuite_env:
            self.robosuite_env.close()
            self.robosuite_env = None


# Actions
class SetJointPositions(Action):
    """Set robot joint positions."""
    
    def __init__(self, positions: List[float]):
        self.positions = positions
    
    def applyTo(self, agent, sim: RobosuiteSimulation):
        """Apply action to agent."""
        if hasattr(sim, 'robots') and agent in sim.robots:
            robot_idx = sim.robots.index(agent)
            if robot_idx < len(sim.robosuite_env.robots):
                sim.pending_robot_action = np.array(self.positions)


class OSCPositionAction(Action):
    """Operational Space Control for end-effector."""
    
    def __init__(self, position_delta: Optional[List[float]] = None,
                 orientation_delta: Optional[List[float]] = None,
                 gripper: Optional[float] = None):
        self.position_delta = position_delta or [0, 0, 0]
        self.orientation_delta = orientation_delta or [0, 0, 0]
        self.gripper = gripper if gripper is not None else 0
    
    def applyTo(self, agent, sim: RobosuiteSimulation):
        """Apply action to agent."""
        if hasattr(sim, 'robots') and agent in sim.robots:
            robot_idx = sim.robots.index(agent)
            if robot_idx < len(sim.robosuite_env.robots):
                if sim.controller_type == 'JOINT_POSITION':
                    action = np.zeros(sim.action_dim)
                    action[:3] = self.position_delta
                else:
                    action = np.zeros(7)
                    action[:3] = self.position_delta
                    action[3:6] = self.orientation_delta
                    action[6] = self.gripper
                
                sim.pending_robot_action = action