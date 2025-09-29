"""RoboSuite Simulator interface for Scenic."""

from typing import Dict, List, Any, Optional, Union
import numpy as np
import os
import tempfile
import shutil
import xml.etree.ElementTree as ET

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


# Deferred class definitions to avoid importing RoboSuite at module load
def _get_arena_class():
    """Get CustomXMLArena class (requires RoboSuite imports)."""
    from robosuite.models.arenas import Arena
    from robosuite.utils.mjcf_utils import xml_path_completion
    
    class CustomXMLArena(Arena):
        """Arena created from custom XML string or file."""
        
        def __init__(self, xml_string, scenic_file_path=None):
            super().__init__(xml_path_completion("arenas/empty_arena.xml"))
            
            # Handle file path vs XML string
            if not xml_string.strip().startswith('<'):
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
            
            custom_worldbody = custom_tree.find(".//worldbody")
            if custom_worldbody is None:
                raise ValueError("Custom XML must have a worldbody element")
            
            # Replace worldbody content
            for child in list(self.worldbody):
                self.worldbody.remove(child)
            
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
    
    return CustomXMLArena


def _get_object_class():
    """Get CustomMeshObject class (requires RoboSuite imports)."""
    from robosuite.models.objects import MujocoXMLObject
    
    class CustomMeshObject(MujocoXMLObject):
        """Custom manipulable object from XML string with automatic joint and collision handling."""
        
        _existing_joint_names = set()
        
        def __init__(self, name, xml_string, scenic_file_path=None):
            self.object_name = name
            self.temp_dir = tempfile.mkdtemp()
            
            base_dir = scenic_file_path if scenic_file_path else os.getcwd()
            
            # Handle file path vs XML string
            if not xml_string.strip().startswith('<'):
                if not os.path.isabs(xml_string):
                    xml_path = os.path.join(base_dir, xml_string)
                else:
                    xml_path = xml_string
                
                if os.path.exists(xml_path):
                    with open(xml_path, 'r') as f:
                        xml_string = f.read()
                    base_dir = os.path.dirname(xml_path)
                else:
                    raise FileNotFoundError(f"Object XML file not found: {xml_path}")
            
            tree = ET.fromstring(xml_string)
            
            self._process_asset_files(tree, base_dir)
            tree = self._process_xml_tree(tree, name)
            
            fixed_xml = ET.tostring(tree, encoding='unicode')
            
            temp_xml_path = os.path.join(self.temp_dir, f"{name}.xml")
            with open(temp_xml_path, 'w') as f:
                f.write(fixed_xml)
            
            try:
                super().__init__(
                    temp_xml_path,
                    name=name,
                    joints=None,
                    obj_type="all",
                    duplicate_collision_geoms=False
                )
            finally:
                pass  # Keep temp_dir for MuJoCo to access files
        
        def _process_asset_files(self, tree, base_dir):
            """Copy mesh and texture files to temp directory."""
            # Process mesh files
            for mesh in tree.findall(".//mesh"):
                file_attr = mesh.get("file")
                if file_attr:
                    if not os.path.isabs(file_attr):
                        full_path = os.path.join(base_dir, file_attr)
                    else:
                        full_path = file_attr
                    
                    if os.path.exists(full_path):
                        filename = os.path.basename(full_path)
                        temp_path = os.path.join(self.temp_dir, filename)
                        shutil.copy2(full_path, temp_path)
                        mesh.set("file", filename)
                        
                        # Auto-copy MTL file for OBJ meshes
                        if full_path.endswith('.obj'):
                            mtl_path = full_path.replace('.obj', '.mtl')
                            if os.path.exists(mtl_path):
                                mtl_filename = os.path.basename(mtl_path)
                                temp_mtl_path = os.path.join(self.temp_dir, mtl_filename)
                                shutil.copy2(mtl_path, temp_mtl_path)
            
            # Process texture files
            for texture in tree.findall(".//texture"):
                file_attr = texture.get("file")
                if file_attr and not texture.get("builtin"):
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
                        
                        texture.set("file", filename)
        
        def _process_xml_tree(self, tree, name):
            """Ensure joint and collision geometry."""
            # Find object body
            object_body = tree.find(".//body[@name='object']")
            if object_body is None:
                worldbody = tree.find(".//worldbody")
                if worldbody is not None:
                    for body in worldbody.findall("body"):
                        inner_body = body.find("body[@name='object']")
                        if inner_body is not None:
                            object_body = inner_body
                            break
            
            if object_body is None:
                return tree
            
            self._ensure_free_joint(object_body, name)
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
                
                joint = ET.Element("joint")
                joint.set("name", joint_name)
                joint.set("type", "free")
                joint.set("damping", "0.0005")
                
                object_body.insert(0, joint)
                self._existing_joint_names.add(joint_name)
                self.joint_name = joint_name
        
        def _ensure_collision_geometry(self, object_body, name):
            """Ensure proper collision geometry exists."""
            has_proper_collision = False
            mesh_collision_geom = None
            visual_geom = None
            
            for geom in object_body.findall("geom"):
                geom_group = geom.get("group", "0")
                geom_type = geom.get("type", "box")
                
                if geom_group == "0":
                    if geom_type == "mesh":
                        mesh_collision_geom = geom
                    else:
                        has_proper_collision = True
                        break
                elif geom_group == "1":
                    visual_geom = geom
            
            # Convert mesh collision to visual + box collision
            if mesh_collision_geom is not None and not has_proper_collision:
                mesh_collision_geom.set("group", "1")
                mesh_collision_geom.set("contype", "0")
                mesh_collision_geom.set("conaffinity", "0")
                
                collision_geom = ET.Element("geom")
                collision_geom.set("name", "collision_auto")
                collision_geom.set("type", "box")
                collision_geom.set("size", "0.04 0.04 0.04")
                
                pos = mesh_collision_geom.get("pos", "0 0 0")
                collision_geom.set("pos", pos)
                
                collision_geom.set("group", "0")
                collision_geom.set("rgba", "0 0 0 0")
                
                for prop in ["solimp", "solref", "density", "friction"]:
                    value = mesh_collision_geom.get(prop)
                    if value:
                        collision_geom.set(prop, value)
                
                if not collision_geom.get("solimp"):
                    collision_geom.set("solimp", "0.998 0.998 0.001")
                if not collision_geom.get("solref"):
                    collision_geom.set("solref", "0.001 1")
                if not collision_geom.get("density"):
                    collision_geom.set("density", "100")
                if not collision_geom.get("friction"):
                    collision_geom.set("friction", "0.95 0.3 0.1")
                
                index = list(object_body).index(mesh_collision_geom) + 1
                object_body.insert(index, collision_geom)
                return
            
            # Create collision if none exists
            if not has_proper_collision:
                if visual_geom is not None:
                    collision_geom = ET.Element("geom")
                    collision_geom.set("name", "collision_auto")
                    
                    geom_type = visual_geom.get("type", "box")
                    pos = visual_geom.get("pos", "0 0 0")
                    
                    if geom_type == "mesh":
                        collision_geom.set("type", "box")
                        collision_geom.set("size", "0.04 0.04 0.04")
                    else:
                        collision_geom.set("type", geom_type)
                        size = visual_geom.get("size")
                        if size:
                            collision_geom.set("size", size)
                    
                    collision_geom.set("pos", pos)
                    
                else:
                    collision_geom = ET.Element("geom")
                    collision_geom.set("name", "collision_default")
                    collision_geom.set("type", "box")
                    collision_geom.set("size", "0.04 0.04 0.04")
                    collision_geom.set("pos", "0 0 0")
                
                collision_geom.set("group", "0")
                collision_geom.set("rgba", "0 0 0 0")
                collision_geom.set("solimp", "0.998 0.998 0.001")
                collision_geom.set("solref", "0.001 1")
                collision_geom.set("density", "100")
                collision_geom.set("friction", "0.95 0.3 0.1")
                
                if visual_geom is not None:
                    index = list(object_body).index(visual_geom) + 1
                    object_body.insert(index, collision_geom)
                else:
                    object_body.append(collision_geom)
        
        def __del__(self):
            """Clean up temp directory."""
            if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
                shutil.rmtree(self.temp_dir)
    
    return CustomMeshObject


def _get_env_class():
    """Get ScenicManipulationEnv class (requires RoboSuite imports)."""
    from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
    from robosuite.models.arenas import EmptyArena, MultiTableArena
    from robosuite.models.tasks import ManipulationTask
    from robosuite.utils.observables import Observable, sensor
    
    class ScenicManipulationEnv(ManipulationEnv):
        """Scenic-driven manipulation environment."""
        
        def __init__(self, scenic_sim, **kwargs):
            self.scenic_sim = scenic_sim
            super().__init__(**kwargs)
        
        def _load_model(self):
            """Load models and create arena."""
            super()._load_model()
            
            # Position robots directly from scenic_sim
            for i, robot in enumerate(self.scenic_sim.robots):
                if i < len(self.robots):
                    pos = [robot.position.x, robot.position.y, 0]
                    self.robots[i].robot_model.set_base_xpos(pos)
            
            mujoco_arena = self._create_arena()
            mujoco_arena.set_origin([0, 0, 0])
            
            self.mujoco_objects = self._create_objects()
            
            self.model = ManipulationTask(
                mujoco_arena=mujoco_arena,
                mujoco_robots=[robot.robot_model for robot in self.robots],
                mujoco_objects=self.mujoco_objects
            )
        
        def _create_arena(self):
            """Create arena based on scenic objects."""
            # Check for custom arena
            for obj in self.scenic_sim.objects:
                if hasattr(obj, 'isCustomArena') and obj.isCustomArena:
                    xml_string = getattr(obj, 'arenaXml', '')
                    scenic_file_path = str(self.scenic_sim.scene.params.get('scenic_file_dir', ''))
                    return CustomXMLArena(xml_string, scenic_file_path)
            
            # Check for tables
            tables = [obj for obj in self.scenic_sim.objects 
                    if hasattr(obj, 'isTable') and obj.isTable]
            
            if tables:
                table_offsets = []
                for t in tables:
                    table_offsets.append([t.position.x, t.position.y, 0.8])
                
                return MultiTableArena(
                    table_offsets=table_offsets,
                    table_full_sizes=[(t.width, t.length, 0.05) for t in tables],
                    has_legs=[True] * len(tables)
                )
            
            return EmptyArena()
        
        def _create_objects(self) -> List:
            """Create Robosuite objects from Scenic objects."""
            mujoco_objects = []
            scenic_file_path = str(self.scenic_sim.scene.params.get('scenic_file_dir', ''))
            
            for obj in self.scenic_sim.objects:
                # Skip non-manipulable objects
                if (hasattr(obj, '_isArenaComponent') and obj._isArenaComponent) or \
                hasattr(obj, 'robotType') or \
                hasattr(obj, 'isCustomArena') or \
                hasattr(obj, 'isTable'):
                    continue
                
                # Generate unique name
                scenic_name = getattr(obj, 'name', None)
                if hasattr(obj, 'objectType') and obj.objectType == 'MJCF':
                    obj_name = f"{scenic_name}_{len(mujoco_objects)}" if scenic_name else f"mjcf_object_{len(mujoco_objects)}"
                    mj_obj = CustomMeshObject(
                        name=obj_name,
                        xml_string=getattr(obj, 'mjcfXml', ''),
                        scenic_file_path=scenic_file_path
                    )
                elif hasattr(obj, 'makeRobosuiteObject'):
                    obj_name = f"{scenic_name}_{len(mujoco_objects)}" if scenic_name else f"{obj.__class__.__name__}_{len(mujoco_objects)}"
                    mj_obj = obj.makeRobosuiteObject(obj_name)
                else:
                    continue
                
                # Store mapping
                self.scenic_sim.object_name_map[obj] = obj_name
                mujoco_objects.append(mj_obj)
            
            return mujoco_objects
        
        def _reset_internal(self):
            """Reset environment internals."""
            super()._reset_internal()
            
            # Set initial positions
            for obj in self.scenic_sim.objects:
                obj_name = self.scenic_sim.object_name_map.get(obj)
                if not obj_name:
                    continue
                    
                # Find corresponding mujoco object
                mj_obj = None
                for mobj in self.mujoco_objects:
                    if mobj.name == obj_name:
                        mj_obj = mobj
                        break
                
                if mj_obj:
                    pos = [obj.position.x, obj.position.y, obj.position.z]
                    quat = [1, 0, 0, 0]  # Default quaternion
                    
                    # Find joint name
                    joint_name = None
                    possible_names = []
                    
                    if hasattr(mj_obj, 'joint_name'):
                        base_joint = mj_obj.joint_name
                        possible_names.append(f"{mj_obj.name}_{base_joint}")
                        possible_names.append(base_joint)
                    
                    if hasattr(mj_obj, 'joints') and mj_obj.joints:
                        possible_names.extend(mj_obj.joints)
                    
                    possible_names.append(f"{mj_obj.name}_joint0")
                    possible_names.append(f"{mj_obj.name}_object_joint")
                    
                    for name in possible_names:
                        if name in self.sim.model.joint_names:
                            joint_name = name
                            break
                    
                    if joint_name:
                        self.sim.data.set_joint_qpos(
                            joint_name,
                            np.concatenate([np.array(pos), np.array(quat)])
                        )
            
            self.sim.forward()
            self.sim.step()
        
        def _setup_references(self):
            """Setup references to simulation objects."""
            super()._setup_references()
            # Let scenic_sim handle body mapping
            for obj in self.mujoco_objects:
                try:
                    if hasattr(obj, 'root_body'):
                        body_name = obj.root_body
                    else:
                        body_name = obj.name
                    
                    body_id = self.sim.model.body_name2id(body_name)
                    self.scenic_sim.body_id_map[obj.name] = body_id
                except:
                    for suffix in ['_main', '_body', '']:
                        try:
                            body_name = f"{obj.name}{suffix}"
                            body_id = self.sim.model.body_name2id(body_name)
                            self.scenic_sim.body_id_map[obj.name] = body_id
                            break
                        except:
                            continue
        
        def reward(self, action=None) -> float:
            """Compute reward."""
            return 0.0
        
        def _check_success(self) -> bool:
            """Check task success."""
            return False

    return ScenicManipulationEnv


class RobosuiteSimulator(Simulator):

    """Simulator for RoboSuite robotic manipulation environments.
    
    Args:
        render: Whether to render the simulation visually. If True, opens a 
            viewer window showing the simulation. Default True.
        real_time: Whether to run the simulation in real-time. If False, runs
            as fast as possible. Default True.
        speed: Speed multiplier for real-time simulation. Values > 1.0 speed up
            the simulation, < 1.0 slow it down. Only used when real_time=True.
            Default 1.0.
        env_config: Additional configuration passed to RoboSuite environment.
            Can include settings like 'control_freq', 'horizon', etc. Default
            empty dict.
        controller_config: Robot controller configuration. If None, uses 
            RoboSuite's default controller for the robot type. Can specify
            controller type and parameters. Default None.
        camera_view: Name of camera to use for rendering. Options include
            'frontview', 'birdview', 'agentview', 'sideview', 'robot0_robotview',
            'robot0_eye_in_hand'. Default None (uses RoboSuite default).
        lite_physics: Whether to use simplified physics for faster simulation.
            Reduces physics accuracy but improves performance. Default None
            (uses RoboSuite default).
    """

    def __init__(self, render: bool = True, real_time: bool = True, speed: float = 1.0,
                 env_config: Optional[Dict] = None, controller_config: Optional[Dict] = None,
                 camera_view: Optional[str] = None, lite_physics: Optional[bool] = None):
        super().__init__()
        
        # Import RoboSuite only when creating simulator
        try:
            global mujoco, suite
            import mujoco
            import robosuite as suite
        except ImportError as e:
            raise RuntimeError(f"Unable to import RoboSuite: {e}")
        
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
        
        # Load deferred classes
        global CustomXMLArena, CustomMeshObject, ScenicManipulationEnv
        CustomXMLArena = _get_arena_class()
        CustomMeshObject = _get_object_class()
        ScenicManipulationEnv = _get_env_class()
        
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.env_config = env_config or {}
        self.controller_config = controller_config
        self.camera_view = camera_view
        self.lite_physics = lite_physics
        
        self.robosuite_env = None
        self.model = None
        self.data = None
        
        self.body_id_map = {}
        self.object_name_map = {}
        self.robots = []
        self.prev_positions = {}
        self._current_obs = None
        
        self.pending_robot_action = None
        self.action_dim = DEFAULT_ACTION_DIM
        self.controller_type = None
        
        self.timestep = kwargs.get('timestep') or DEFAULT_TIMESTEP
        self.physics_timestep = kwargs.get('physics_timestep') or DEFAULT_PHYSICS_TIMESTEP
        self.physics_steps = int(self.timestep / self.physics_timestep)
        self.agents = []
        
        super().__init__(scene, **kwargs)
    

    def setup(self):
        """Initialize the RoboSuite environment."""
        super().setup()
        
        # Extract robots for environment
        robot_types = []
        for obj in self.objects:
            if hasattr(obj, 'robotType'):
                robot_types.append(obj.robotType)
                self.robots.append(obj)
        
        if not robot_types:
            raise ValueError("At least one robot is required in the scene")
        
        robot_arg = robot_types[0] if len(robot_types) == 1 else robot_types
        
        env_kwargs = {
            'scenic_sim': self,  # Pass self directly
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
        self._detect_controller_type()
        
        self._finalize_setup()


    def _finalize_setup(self):
        """Common setup after environment creation."""
        self.model = self.robosuite_env.sim.model._model
        self.data = self.robosuite_env.sim.data._data
        
        if self.render and self.camera_view is not None:
            self._set_camera_view()
        
        self._setup_body_mapping()
        
        self.agents = [obj for obj in self.objects 
                      if hasattr(obj, 'behavior') and obj.behavior]
    
    def _set_camera_view(self):
        """Set the camera view if specified."""
        camera_id = (CAMERA_VIEWS.get(self.camera_view.lower(), 0) 
                    if isinstance(self.camera_view, str) 
                    else self.camera_view)
        if self.robosuite_env.viewer:
            self.robosuite_env.viewer.set_camera(camera_id=camera_id)
    
    def _extract_scenic_config(self) -> Dict:
        """Extract configuration from Scenic scene."""
        config = {'robots': [], 'tables': [], 'objects': [], 'custom_arena': None}
        
        scenic_file_path = None
        
        if 'scenic_file_dir' in self.scene.params:
            path_obj = self.scene.params.get('scenic_file_dir')
            if path_obj:
                scenic_file_path = str(path_obj)
        
        if not scenic_file_path:
            import sys
            for arg in sys.argv:
                if arg.endswith('.scenic'):
                    scenic_file_abspath = os.path.abspath(arg)
                    if os.path.exists(scenic_file_abspath):
                        scenic_file_path = os.path.dirname(scenic_file_abspath)
                        break
        
        if not scenic_file_path:
            scenic_file_path = os.getcwd()
        
        for obj in self.objects:
            if hasattr(obj, '_isArenaComponent') and obj._isArenaComponent:
                continue
                
            if hasattr(obj, 'robotType'):
                self._add_robot_config(config['robots'], obj)
            elif hasattr(obj, 'isCustomArena') and obj.isCustomArena:
                config['custom_arena'] = {
                    'xml': getattr(obj, 'arenaXml', ''),
                    'scenic_file_path': scenic_file_path
                }
            elif hasattr(obj, 'isTable') and obj.isTable:
                self._add_table_config(config['tables'], obj)
            elif hasattr(obj, 'makeRobosuiteObject') or (hasattr(obj, 'objectType') and obj.objectType == 'MJCF'):
                self._add_object_config(config['objects'], obj, scenic_file_path)
        
        return config
    
    def _add_robot_config(self, robots: List, obj):
        """Add robot configuration."""
        robots.append({
            'type': obj.robotType,
            'position': [obj.position.x, obj.position.y, 0] 
        })
        self.robots.append(obj)
    
    def _add_table_config(self, tables: List, obj):
        """Add table configuration."""
        tables.append({
            'position': [obj.position.x, obj.position.y, obj.position.z],
            'size': (obj.width, obj.length, 0.05)  # MultiTableArena tabletop thickness
        })
    
    def _add_object_config(self, objects: List, obj, scenic_file_path):
        """Add object configuration."""
        scenic_name = getattr(obj, 'name', None)
        
        if hasattr(obj, 'objectType') and obj.objectType == 'MJCF':
            obj_name = f"{scenic_name}_{len(objects)}" if scenic_name else f"mjcf_object_{len(objects)}"
            obj_type = 'MJCF'
        else:
            base_name = scenic_name if scenic_name else obj.__class__.__name__
            obj_name = f"{base_name}_{len(objects)}"
            obj_type = obj.__class__.__name__
        
        config = {
            'type': obj_type,
            'name': obj_name,
            'position': [obj.position.x, obj.position.y, obj.position.z],
            'color': getattr(obj, 'color', [1, 0, 0, 1]),
            'scenic_obj': obj  # Store reference to the Scenic object
        }
        
        self.object_name_map[obj] = obj_name
        
        if obj_type == 'MJCF':
            config['mjcfXml'] = getattr(obj, 'mjcfXml', '')
            config['scenic_file_path'] = scenic_file_path
        
        objects.append(config)
    
    def _get_robot_arg(self, robots: List) -> Union[str, List[str]]:
        """Get robot argument for environment creation."""
        if not robots:
            raise ValueError("At least one robot is required")
        return ([r['type'] for r in robots] if len(robots) > 1 
                else robots[0]['type'])
    
    def _detect_controller_type(self):
        """Detect controller type from first robot."""
        if self.robots and self.robosuite_env.robots:
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
        
        for name, body_id in self.body_id_map.items():
            self.prev_positions[name] = self.data.xpos[body_id].copy()
        
        action = (self.pending_robot_action if self.pending_robot_action is not None 
                 else np.zeros(self.action_dim))
        
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
            elif prop == 'jointPositions' and robot_idx is not None:
                values[prop] = self._get_robot_joints(robot_idx)
            elif prop == 'eefPos' and robot_idx is not None:
                values[prop] = self._get_eef_position(robot_idx)
            elif prop == 'gripperState' and robot_idx is not None:
                values[prop] = self._get_gripper_state(robot_idx)
            else:
                values[prop] = getattr(obj, prop, None)
        
        return values
    
    def _get_object_position(self, obj) -> Vector:
        obj_name = self.object_name_map.get(obj)
        if obj_name and obj_name in self.body_id_map:
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