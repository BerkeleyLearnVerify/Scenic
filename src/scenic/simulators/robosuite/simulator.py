# src/scenic/simulators/robosuite/simulator.py

"""RoboSuite Simulator interface for Scenic."""

from typing import Dict, List, Any, Optional, Union
import numpy as np
import mujoco

try:
    import robosuite as suite
    from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
    from robosuite.models.arenas import TableArena, MultiTableArena
    from robosuite.models.tasks import ManipulationTask
    from robosuite.models.objects import (
        BallObject, BoxObject, CylinderObject, CapsuleObject,
        MilkObject, CerealObject, CanObject, BreadObject,
        HammerObject, SquareNutObject, RoundNutObject, BottleObject
    )
    from robosuite.utils.placement_samplers import UniformRandomSampler
    from robosuite.utils.observables import Observable, sensor
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

# Object type mapping
OBJECT_FACTORIES = {
    'Ball': lambda cfg: BallObject(
        name=cfg['name'],
        size=[cfg.get('radius', cfg.get('size', [0.02])[0])],
        rgba=cfg.get('color', [1, 0, 0, 1])
    ),
    'Box': lambda cfg: BoxObject(
        name=cfg['name'],
        size=cfg.get('size', [0.025, 0.025, 0.025]),
        rgba=cfg.get('color', [1, 0, 0, 1])
    ),
    'Cylinder': lambda cfg: CylinderObject(
        name=cfg['name'],
        size=_extract_cylinder_size(cfg)
    ),
    'Capsule': lambda cfg: CapsuleObject(
        name=cfg['name'],
        size=_extract_cylinder_size(cfg)
    ),
    'Milk': lambda cfg: MilkObject(name=cfg['name']),
    'Cereal': lambda cfg: CerealObject(name=cfg['name']),
    'Can': lambda cfg: CanObject(name=cfg['name']),
    'Bread': lambda cfg: BreadObject(name=cfg['name']),
    'Hammer': lambda cfg: HammerObject(name=cfg['name']),
    'SquareNut': lambda cfg: SquareNutObject(name=cfg['name']),
    'RoundNut': lambda cfg: RoundNutObject(name=cfg['name']),
    'Bottle': lambda cfg: BottleObject(name=cfg['name'])
}


def _extract_cylinder_size(config: Dict) -> List[float]:
    """Extract [radius, height] for cylinder-like objects."""
    size = config.get('size', [0.02, 0.04])
    if len(size) == 3:  # Convert from [width, length, height]
        return [size[0] / 2, size[2]]
    return size


class CustomManipulationEnv(ManipulationEnv):
    """Custom manipulation environment for Scenic-defined scenarios."""
    
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
            from robosuite.models.arenas import EmptyArena
            return EmptyArena()
        
        # Use MultiTableArena for any number of tables (1+)
        from robosuite.models.arenas import MultiTableArena
        return MultiTableArena(
            table_offsets=[t.get('position', [0, 0, 0.8]) for t in self.scenic_tables],
            table_full_sizes=[t.get('size', (0.8, 0.8, 0.05)) for t in self.scenic_tables],
            has_legs=[True] * len(self.scenic_tables)
        )
    
    def _create_objects(self) -> List:
        """Create Robosuite objects from Scenic configuration."""
        mujoco_objects = []
        
        for obj_config in self.scenic_objects:
            obj_type = obj_config['type']
            
            # Create object using factory or default to box
            factory = OBJECT_FACTORIES.get(obj_type, OBJECT_FACTORIES['Box'])
            mj_obj = factory(obj_config)
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
                self.sim.data.set_joint_qpos(
                    mj_obj.joints[0],
                    np.concatenate([np.array(pos), np.array(quat)])
                )
    
    def reward(self, action=None) -> float:
        """Compute reward."""
        return 0.0
    
    def _check_success(self) -> bool:
        """Check task success."""
        return False


class RobosuiteSimulator(Simulator):
    """Simulator for RoboSuite environments."""
    
    def __init__(self, render: bool = True, real_time: bool = True, speed: float = 1.0,
                 use_environment: Optional[str] = None, env_config: Optional[Dict] = None,
                 controller_config: Optional[Dict] = None, camera_view: Optional[str] = None,
                 lite_physics: Optional[bool] = None):
        super().__init__()
        if suite is None:
            raise RuntimeError(f"Unable to import RoboSuite: {_import_error}")
        
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.use_environment = use_environment
        self.env_config = env_config or {}
        self.controller_config = controller_config
        self.camera_view = camera_view
        self.lite_physics = lite_physics
    
    def createSimulation(self, scene, **kwargs):
        """Create a simulation instance."""
        return RobosuiteSimulation(
            scene, self.render, self.real_time, self.speed,
            self.use_environment, self.env_config,
            self.controller_config, self.camera_view,
            self.lite_physics, **kwargs
        )


class RobosuiteSimulation(Simulation):
    """Simulation for RoboSuite environments."""
    
    def __init__(self, scene, render: bool, real_time: bool, speed: float,
                 use_environment: Optional[str], env_config: Optional[Dict],
                 controller_config: Optional[Dict], camera_view: Optional[str],
                 lite_physics: Optional[bool], **kwargs):
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.use_environment = use_environment
        self.env_config = env_config or {}
        self.controller_config = controller_config
        self.camera_view = camera_view
        self.lite_physics = lite_physics
        
        # Environment state
        self.robosuite_env = None
        self.model = None
        self.data = None
        
        # Object tracking - maps Scenic object to its name
        self.body_id_map = {}
        self.object_name_map = {}  # Maps Scenic object to its name in the sim
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
        
        if not self.use_environment:
            raise ValueError("Environment name must be specified via 'use_environment' parameter")
        
        if self.use_environment.lower() == "custom":
            self._setup_custom_environment()
        else:
            self._setup_standard_environment()
        
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
    
    def _setup_custom_environment(self):
        """Setup custom manipulation environment."""
        scenic_config = self._extract_scenic_config()
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
        
        self.robosuite_env = CustomManipulationEnv(**env_kwargs)
        self._current_obs = self.robosuite_env.reset()
        self._detect_controller_type(scenic_config['robots'])
    
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
        # Use the Scenic object's name attribute directly
        obj_name = getattr(obj, 'name', f"obj_{id(obj)}")
        
        config = {
            'type': obj.objectType,
            'name': obj_name,  # Use Scenic's name directly
            'position': [obj.position.x, obj.position.y, obj.position.z],
            'color': getattr(obj, 'color', [1, 0, 0, 1])
        }
        
        # Store mapping
        self.object_name_map[obj] = obj_name
        
        # Add size/radius info
        if hasattr(obj, 'radius'):
            config['radius'] = obj.radius
        elif hasattr(obj, 'width'):
            config['size'] = [obj.width, obj.length, obj.height]
        
        objects.append(config)
    
    def _get_robot_arg(self, robots: List) -> Union[str, List[str]]:
        """Get robot argument for environment creation."""
        if not robots:
            return "Panda"
        return ([r['type'] for r in robots] if len(robots) > 1 
                else robots[0]['type'])
    
    def _setup_standard_environment(self):
        """Setup standard RoboSuite environment."""
        robots = [obj for obj in self.objects if hasattr(obj, 'robot_type')]
        robot_arg = self._get_robot_arg([{'type': r.robot_type} for r in robots])
        
        config = {
            **self.env_config,
            'has_renderer': self.render,
            'render_camera': self.camera_view,
            'camera_names': ["frontview", "robot0_eye_in_hand"],
            'controller_configs': self.controller_config
        }
        
        self.robosuite_env = suite.make(self.use_environment, robots=robot_arg, **config)
        self._current_obs = self.robosuite_env.reset()
        
        # Store robot references
        for i, robot in enumerate(robots[:len(self.robosuite_env.robots)]):
            self.robots.append(robot)
            robot_name = f"robot{i}"
            self.object_name_map[robot] = robot_name
        
        # Map standard environment objects using their Scenic name
        for obj in self.objects:
            if hasattr(obj, 'name') and not hasattr(obj, 'robot_type'):
                # For standard environments, use the object's Scenic name
                self.object_name_map[obj] = obj.name
        
        self._apply_initial_positions()
        self._detect_controller_type([{'type': r.robot_type} for r in robots])
    
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
    
    def _apply_initial_positions(self):
        """Apply initial positions for environment objects."""
        for obj in self.objects:
            obj_name = self.object_name_map.get(obj)
            if obj_name and hasattr(obj, 'position'):
                self._set_object_position(obj, obj_name)
        
        self.robosuite_env.sim.forward()
        
        # Let physics settle
        for _ in range(PHYSICS_SETTLE_STEPS):
            self.robosuite_env.step(np.zeros(self.action_dim))
        
        self._current_obs = self.robosuite_env._get_observations()
    
    def _set_object_position(self, obj, obj_name):
        """Set position for single object."""
        joint_name = f"{obj_name}{JOINT_SUFFIX}"
        try:
            qpos = np.concatenate([
                [obj.position.x, obj.position.y, obj.position.z],
                [1, 0, 0, 0]
            ])
            self.robosuite_env.sim.data.set_joint_qpos(joint_name, qpos)
        except:
            pass
    
    def createObjectInSimulator(self, obj):
        """Required by Scenic's Simulator interface."""
        """See _setup_custom_environment() and _setup_standard_environment() for object creation."""
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