# src/scenic/simulators/robosuite/simulator.py

"""RoboSuite Simulator interface for Scenic - Custom Environment Only."""

from typing import Dict, List, Any, Optional, Union
import numpy as np
import mujoco
import os
import tempfile
import xml.etree.ElementTree as ET

try:
    import robosuite as suite
    from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
    from robosuite.models.arenas import EmptyArena, MultiTableArena
    from robosuite.models.tasks import ManipulationTask
    from robosuite.models.objects import (
        BallObject, BoxObject, CylinderObject, CapsuleObject,
        MilkObject, CerealObject, CanObject, BreadObject,
        HammerObject, SquareNutObject, RoundNutObject, BottleObject,
        MujocoGeneratedObject
    )
    from robosuite.utils.observables import Observable, sensor
    from robosuite.utils.mjcf_utils import array_to_string
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

# RoboSuite naming patterns
BODY_SUFFIXES = ['_main', '_body', '']
JOINT_SUFFIX = '_joint0'

# Camera view mapping
CAMERA_VIEWS = {
    "frontview": 0,
    "birdview": 1,
    "agentview": 2,
    "sideview": 3,
    "robot0_robotview": 4,
    "robot0_eye_in_hand": 5
}


class CustomMJCFObject(MujocoGeneratedObject):
    """Custom MJCF object from user-provided XML."""
    
    def __init__(self, name, geom_xml, rgba=None):
        super().__init__()
        self._name = name
        self._joints = [f"{name}_joint0"]
        self.geom_xml = geom_xml
        self.rgba = rgba or [1, 0, 0, 1]
        self._obj = self._get_object_subtree()
    
    @property
    def name(self):
        return self._name
    
    @property
    def joints(self):
        return self._joints
    
    @property
    def root_body(self):
        return self._name
    
    @property
    def horizontal_radius(self):
        return 0.03
    
    @property
    def visual_geoms(self):
        """Return visual geometry names."""
        return [f"{self._name}_visual"]

    @property
    def contact_geoms(self):
        """Return contact geometry names."""
        return [f"{self._name}_collision"]
    
    @property
    def sites(self):
        """Return site names."""
        return []  # MJCF objects typically don't have sites
    
    def _sanitize_element(self, elem):
        for key in list(elem.attrib.keys()):
            val = elem.attrib[key]
            if val is None:
                elem.attrib[key] = ""
            elif not isinstance(val, str):
                elem.attrib[key] = str(val)
        for child in elem:
            self._sanitize_element(child)
    
    def _get_object_subtree(self):
        obj = ET.Element("body")
        obj.set("name", self._name)
    
        joint = ET.SubElement(obj, "joint")
        joint.set("name", self._joints[0])
        joint.set("type", "free")
    
        try:
            if self.geom_xml.strip().startswith('<geom'):
                geom_str = self.geom_xml.strip()
                if not geom_str.endswith('>'):
                    geom_str += '/>'
                elif not geom_str.endswith('/>') and not geom_str.endswith('</geom>'):
                    geom_str = geom_str[:-1] + '/>'
            
                # Parse user's geom as template
                template = ET.fromstring(geom_str)
            
                # Create collision geom
                col_geom = ET.SubElement(obj, "geom")
                col_geom.set("name", f"{self._name}_collision")
                for key, value in template.attrib.items():
                    if key not in ['name', 'rgba', 'contype', 'conaffinity', 'group']:
                        col_geom.set(key, value)
                col_geom.set("group", "0")
            
                # Create visual geom
                vis_geom = ET.SubElement(obj, "geom")
                vis_geom.set("name", f"{self._name}_visual")
                for key, value in template.attrib.items():
                    if key not in ['name', 'contype', 'conaffinity', 'group']:
                        vis_geom.set(key, value)
                if 'rgba' not in vis_geom.attrib:
                    vis_geom.set('rgba', array_to_string(self.rgba))
                vis_geom.set("contype", "0")
                vis_geom.set("conaffinity", "0")
                vis_geom.set("group", "1")
            else:
                raise ValueError("Invalid XML")
        except:
            # Fallback default
            col_geom = ET.SubElement(obj, "geom")
            col_geom.set("name", f"{self._name}_collision")
            col_geom.set("type", "box")
            col_geom.set("size", "0.02 0.02 0.02")
            col_geom.set("group", "0")
        
            vis_geom = ET.SubElement(obj, "geom")
            vis_geom.set("name", f"{self._name}_visual")
            vis_geom.set("type", "box")
            vis_geom.set("size", "0.02 0.02 0.02")
            vis_geom.set("rgba", array_to_string(self.rgba))
            vis_geom.set("contype", "0")
            vis_geom.set("conaffinity", "0")
            vis_geom.set("group", "1")
    
        inertial = ET.SubElement(obj, "inertial")
        inertial.set("pos", "0 0 0")
        inertial.set("mass", "0.01")
        inertial.set("diaginertia", "0.001 0.001 0.001")
    
        self._sanitize_element(obj)
        return obj


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
    return [0.02, 0.04]  # Default


def _create_mjcf_object(config: Dict):
    """Create a custom MJCF object from XML."""
    xml_content = config.get('mjcf_xml', '')
    obj_name = config.get('name', 'custom_object')
    rgba = config.get('color', [1, 0, 0, 1])
    
    if xml_content.endswith('.xml') and os.path.exists(xml_content):
        # Load from file
        with open(xml_content, 'r') as f:
            content = f.read()
            # Extract geom elements from full XML
            try:
                root = ET.fromstring(content)
                geoms = root.findall('.//geom')
                if geoms:
                    xml_content = ''.join(ET.tostring(g, encoding='unicode') for g in geoms)
                else:
                    xml_content = content
            except:
                xml_content = content
    
    return CustomMJCFObject(name=obj_name, geom_xml=xml_content, rgba=rgba)


# Object type mapping
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
    # Complex objects with fixed sizes - no size parameter
    'Milk': lambda cfg: MilkObject(name=cfg['name']),
    'Cereal': lambda cfg: CerealObject(name=cfg['name']),
    'Can': lambda cfg: CanObject(name=cfg['name']),
    'Bread': lambda cfg: BreadObject(name=cfg['name']),
    'Hammer': lambda cfg: HammerObject(name=cfg['name']),
    'SquareNut': lambda cfg: SquareNutObject(name=cfg['name']),
    'RoundNut': lambda cfg: RoundNutObject(name=cfg['name']),
    'Bottle': lambda cfg: BottleObject(name=cfg['name']),
    # Custom MJCF objects
    'MJCF': lambda cfg: _create_mjcf_object(cfg)
}


class ScenicManipulationEnv(ManipulationEnv):
    """Scenic-driven manipulation environment."""
    
    def __init__(self, scenic_config: Dict, **kwargs):
        self.scenic_config = scenic_config
        self.scenic_objects = scenic_config.get('objects', [])
        self.scenic_tables = scenic_config.get('tables', [])
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
    
    def _create_arena(self):
        """Create arena based on table configuration."""
        if not self.scenic_tables:
            # No tables - use empty arena (just floor)
            return EmptyArena()
        
        # Use MultiTableArena for any number of tables
        return MultiTableArena(
            table_offsets=[t.get('position', [0, 0, 0.8]) for t in self.scenic_tables],
            table_full_sizes=[t.get('size', (0.8, 0.8, 0.05)) for t in self.scenic_tables],
            has_legs=[True] * len(self.scenic_tables)
        )
    
    def _create_objects(self) -> List:
        """Create Robosuite objects from Scenic configuration."""
        mujoco_objects = []
    
        for i, obj_config in enumerate(self.scenic_objects):
            obj_type = obj_config['type']
            obj_name = obj_config.get('name', f'unnamed_{i}')
            print(f"DEBUG: Creating object {i}: type={obj_type}, name={obj_name}")
        
            factory = OBJECT_FACTORIES.get(obj_type, OBJECT_FACTORIES['Box'])
            mj_obj = factory(obj_config)
        
            # Debug what was actually created
            if hasattr(mj_obj, 'name'):
                print(f"  -> RoboSuite object name: {mj_obj.name}")
            if hasattr(mj_obj, 'joints'):
                print(f"  -> Joints: {mj_obj.joints}")
        
            mujoco_objects.append(mj_obj)
    
        return mujoco_objects
    
    def _setup_references(self):
        """Setup references to simulation objects."""
        super()._setup_references()
        self.obj_body_ids = {
            obj.name: self.sim.model.body_name2id(obj.root_body)
            for obj in self.mujoco_objects
        }
    
    def _setup_observables(self) -> Dict:
        """Setup observables for objects."""
        observables = super()._setup_observables()
        
        for obj in self.mujoco_objects:
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
                joint_name = mj_obj.joints[0] if hasattr(mj_obj, 'joints') else f"{mj_obj.name}_joint0"
            
                print(f"DEBUG: Setting position for {mj_obj.name}: {pos}")
            
                # Set joint position
                self.sim.data.set_joint_qpos(
                    joint_name,
                    np.concatenate([np.array(pos), np.array(quat)])
                )
    
        # Step simulation to apply changes
        self.sim.forward()
        self.sim.step()
    
        # Verify positions
        for mj_obj in self.mujoco_objects:
            body_id = self.sim.model.body_name2id(mj_obj.root_body)
            actual_pos = self.sim.data.body_xpos[body_id]
            print(f"DEBUG: {mj_obj.name} actual position: {actual_pos}")
    
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
    
    def _extract_scenic_config(self) -> Dict:
        """Extract configuration from Scenic scene."""
        config = {'robots': [], 'tables': [], 'objects': []}
        
        for obj in self.objects:
            if hasattr(obj, 'robot_type'):
                self._add_robot_config(config['robots'], obj)
            elif hasattr(obj, 'isTable') and obj.isTable:
                self._add_table_config(config['tables'], obj)
            elif hasattr(obj, 'objectType'):
                self._add_object_config(config['objects'], obj)
        
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
    
    def _add_object_config(self, objects: List, obj):
        """Add object configuration."""
        # Debug logging
        if obj.objectType == 'MJCF':
            print(f"DEBUG: mjcf_name = {repr(getattr(obj, 'mjcf_name', 'NOT_SET'))}")
    
        # Generate unique name - handle all edge cases
        if obj.objectType == 'MJCF':
            mjcf_name = str(getattr(obj, 'mjcf_name', 'custom_object'))
            # Handle various forms of invalid names
            if mjcf_name in ['None', 'none', ''] or mjcf_name.startswith('None'):
                mjcf_name = "custom_object"
            obj_name = f"{mjcf_name}_{len(objects)}"
        else:
            base_name = str(getattr(obj, 'name', obj.objectType))
            if base_name in ['None', 'none', ''] or base_name.startswith('None'):
                base_name = obj.objectType
            obj_name = f"{base_name}_{len(objects)}"
    
        print(f"DEBUG: Creating object with name: {obj_name}")
    
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
            
        for suffix in BODY_SUFFIXES:
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