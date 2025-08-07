"""RoboSuite Simulator interface for Scenic."""

import numpy as np
import mujoco
import time
from typing import Dict, List, Any, Optional

try:        
    import robosuite as suite
    from robosuite.robots import ROBOT_CLASS_MAPPING
except ImportError as e:
    suite = None
    _import_error = e

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector

# Constants
DEFAULT_PHYSICS_TIMESTEP = 0.002
DEFAULT_TIMESTEP = 0.01
DEFAULT_ACTION_DIM = 7
PHYSICS_SETTLE_STEPS = 10
WINDOW_CLOSE_DELAY = 2

# RoboSuite naming patterns
ROBOSUITE_SUFFIXES = {
    'body': ['_main', '_body', ''],
    'joint': '_joint0'
}

# Camera view mapping
CAMERA_VIEWS = {
    "frontview": 0,
    "birdview": 1, 
    "agentview": 2,
    "sideview": 3,
    "robot0_robotview": 4,
    "robot0_eye_in_hand": 5
}


class RobosuiteSimulator(Simulator):
    """Simulator interface for RoboSuite.
    
    Args:
        render: Enable visualization
        real_time: Run simulation in real-time
        speed: Simulation speed multiplier
        use_environment: RoboSuite environment name
        env_config: Additional environment configuration
        controller_config: Robot controller configuration
        camera_view: Camera perspective for rendering
    """
    
    def __init__(self, render=True, real_time=True, speed=1.0, use_environment=None, 
                 env_config=None, controller_config=None, camera_view=None):
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
        
    def createSimulation(self, scene, **kwargs):
        """Create a RoboSuite simulation instance."""
        return RobosuiteSimulation(scene, self.render, self.real_time, self.speed,
                                 self.use_environment, self.env_config, self.controller_config, 
                                 self.camera_view, **kwargs)


class RobosuiteSimulation(Simulation):
    """Simulation instance for RoboSuite.
    
    Manages the interface between Scenic's simulation loop and RoboSuite's
    environment, handling object tracking, action execution, and property updates.
    """
    
    def __init__(self, scene, render: bool, real_time: bool, speed: float, 
                 use_environment: Optional[str] = None,
                 env_config: Optional[Dict] = None, 
                 controller_config: Optional[Dict] = None, 
                 camera_view: Optional[str] = None, **kwargs):
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.use_environment = use_environment
        self.env_config = env_config or {}
        self.controller_config = controller_config
        self.camera_view = camera_view
        self.robosuite_env = None
        self.model = None
        self.data = None
        
        # Object tracking
        self.body_id_map = {}
        self.object_names = {}
        self.robots = []
        self.prev_positions = {}
        self._current_obs = None
        
        # Action handling
        self.pending_robot_action = None
        self.action_dim = DEFAULT_ACTION_DIM
        self.controller_type = None
        
        # Timing
        self.timestep = kwargs.get('timestep') if kwargs.get('timestep') is not None else DEFAULT_TIMESTEP
        self.physics_timestep = kwargs.get('physics_timestep') if kwargs.get('physics_timestep') is not None else DEFAULT_PHYSICS_TIMESTEP
        self.physics_steps = int(self.timestep / self.physics_timestep)
        self.agents = []
        
        super().__init__(scene, **kwargs)
    
    def setup(self):
        """Initialize the RoboSuite environment."""
        super().setup()
        
        if not self.use_environment:
            raise ValueError("Environment name must be specified via 'use_environment' parameter")
            
        # Get robots from scene
        robots = [obj for obj in self.objects if hasattr(obj, 'robot_type')]
        
        # Configure environment
        config = {
            **self.env_config, 
            'has_renderer': self.render, 
            'render_camera': "frontview",
            'camera_names': ["frontview", "robot0_eye_in_hand"], 
            'controller_configs': self.controller_config
        }
        
        # Prepare robots
        robot_arg = "Panda"  # Default
        if robots:
            robot_arg = [r.robot_type for r in robots] if len(robots) > 1 else robots[0].robot_type
        
        # Create environment
        self.robosuite_env = suite.make(self.use_environment, robots=robot_arg, **config)
        self._current_obs = self.robosuite_env.reset()
        self.model = self.robosuite_env.sim.model._model
        self.data = self.robosuite_env.sim.data._data
        
        # Detect controller type and action dimension
        if robots and self.robosuite_env.robots:
            first_robot = self.robosuite_env.robots[0]
            if hasattr(first_robot, 'controller'):
                controller_name = type(first_robot.controller).__name__
                self.controller_type = controller_name
                if hasattr(first_robot.controller, 'control_dim'):
                    self.action_dim = first_robot.controller.control_dim
        
        # Set camera
        if self.render and self.camera_view is not None:
            camera_id = CAMERA_VIEWS.get(self.camera_view.lower(), 0) if isinstance(self.camera_view, str) else self.camera_view
            if self.robosuite_env.viewer:
                self.robosuite_env.viewer.set_camera(camera_id=camera_id)
        
        # Store robot references
        for i, r in enumerate(robots[:len(self.robosuite_env.robots)]):
            self.robots.append(r)
            self.object_names[r] = f"robot{i}"
        
        # Apply initial positions
        self._apply_initial_positions()
        
        # Setup body mapping for environment objects
        self._setup_body_mapping()
        
        # Identify agents
        for obj in self.objects:
            if hasattr(obj, 'behavior') and obj.behavior:
                self.agents.append(obj)
    
    def _setup_body_mapping(self):
        """Map environment objects to MuJoCo body IDs."""
        for obj in self.objects:
            if hasattr(obj, 'envObjectName'):
                # Try to find body with standard naming patterns
                for suffix in ROBOSUITE_SUFFIXES['body']:
                    body_name = f"{obj.envObjectName}{suffix}"
                    try:
                        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                        if body_id != -1:
                            self.body_id_map[obj.envObjectName] = body_id
                            self.prev_positions[obj.envObjectName] = self.data.xpos[body_id].copy()
                            break
                    except:
                        continue
    
    def _apply_initial_positions(self):
        """Apply initial positions for environment objects."""
        for obj in self.objects:
            if hasattr(obj, 'envObjectName') and hasattr(obj, 'position'):
                joint_name = f"{obj.envObjectName}{ROBOSUITE_SUFFIXES['joint']}"
                try:
                    qpos = np.concatenate([
                        [obj.position.x, obj.position.y, obj.position.z],
                        [1, 0, 0, 0]  # identity quaternion
                    ])
                    self.robosuite_env.sim.data.set_joint_qpos(joint_name, qpos)
                except Exception as e:
                    raise RuntimeError(
                        f"Failed to set position for object '{obj.envObjectName}'. "
                        f"Ensure the object exists in the '{self.use_environment}' environment. "
                        f"Error: {e}"
                    )
        
        # Update physics
        self.robosuite_env.sim.forward()
        
        # Let physics settle
        for _ in range(PHYSICS_SETTLE_STEPS):
            self.robosuite_env.step(np.zeros(self.action_dim))
            
        # Get fresh observations
        self._current_obs = self.robosuite_env._get_observations()
    
    def createObjectInSimulator(self, obj):
        """Required by Scenic's Simulator interface but not used.
        
        RoboSuite creates all objects at environment initialization,
        so dynamic object creation is not supported.
        """
        pass
    
    def executeActions(self, allActions: Dict[Any, List]) -> None:
        """Execute actions by calling their applyTo methods."""
        super().executeActions(allActions)
        
        # Clear pending actions
        self.pending_robot_action = None
        
        # Process actions for each agent
        for agent in self.agents:
            if agent in allActions and allActions[agent]:
                for action in allActions[agent]:
                    if action and hasattr(action, 'applyTo'):
                        action.applyTo(agent, self)
                        break  # One action per agent per timestep
    
    def step(self):
        """Step the simulation forward one timestep."""
        if hasattr(self, '_done') and self._done:
            return
            
        # Store previous positions
        for name, body_id in self.body_id_map.items():
            self.prev_positions[name] = self.data.xpos[body_id].copy()
        
        # Use pending action or zeros
        action = self.pending_robot_action if self.pending_robot_action is not None else np.zeros(self.action_dim)
        
        # Step simulation
        for _ in range(self.physics_steps):
            obs, reward, done, info = self.robosuite_env.step(action)
            self._current_obs = obs
            self._done = done
            if self.render:
                self.robosuite_env.render()
            if done:
                break
        
        # Clear pending actions
        self.pending_robot_action = None
    
    def getProperties(self, obj, properties: List[str]) -> Dict[str, Any]:
        """Get current property values for an object."""
        values = {}
        robot_idx = self.robots.index(obj) if obj in self.robots else None
        
        for prop in properties:
            if prop == 'position':
                # For environment objects, get from observations
                if hasattr(obj, 'envObjectName') and self._current_obs:
                    obs_key = f"{obj.envObjectName}_pos"
                    if obs_key in self._current_obs:
                        pos = self._current_obs[obs_key]
                        values[prop] = Vector(pos[0], pos[1], pos[2])
                    else:
                        values[prop] = obj.position
                else:
                    values[prop] = obj.position
                    
            elif prop == 'joint_positions' and robot_idx is not None:
                if robot_idx < len(self.robosuite_env.robots):
                    positions = self.robosuite_env.robots[robot_idx]._joint_positions
                    values[prop] = list(positions)
                else:
                    values[prop] = []
                    
            elif prop == 'eef_pos' and robot_idx is not None and self._current_obs:
                pos = self._current_obs.get(f'robot{robot_idx}_eef_pos', [0, 0, 0])
                values[prop] = list(pos)
                
            elif prop == 'gripper_state' and robot_idx is not None and self._current_obs:
                state = self._current_obs.get(f'robot{robot_idx}_gripper_qpos', [0, 0])
                values[prop] = list(state)
                
            else:
                values[prop] = getattr(obj, prop, None)
        
        return values
    
    def getCurrentObservation(self) -> Optional[Dict]:
        """Get current observation dictionary."""
        return self._current_obs
    
    def checkSuccess(self) -> bool:
        """Check if task is successfully completed."""
        return self.robosuite_env._check_success() if hasattr(self.robosuite_env, '_check_success') else False
    
    def destroy(self):
        """Clean up simulation resources."""
        if self.render and self.robosuite_env:
            print(f"Simulation complete. Window will close in {WINDOW_CLOSE_DELAY} seconds...")
            time.sleep(WINDOW_CLOSE_DELAY)
        
        try:
            if self.robosuite_env:
                self.robosuite_env.close()
                self.robosuite_env = None
        except Exception as e:
            print(f"Warning: Error closing RoboSuite environment: {e}")