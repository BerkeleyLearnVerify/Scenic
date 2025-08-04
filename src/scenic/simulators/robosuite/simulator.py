"""RoboSuite Simulator for Scenic - Version 3.2 (Clean)"""

import numpy as np
import mujoco
import time
import math
from typing import Dict, List, Any

try:        
    import robosuite as suite
    from robosuite.robots import ROBOT_CLASS_MAPPING
except ImportError as e:
    suite = None
    _import_error = e

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector
from .utils import scenic_to_rgba


class RobosuiteSimulator(Simulator):
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
        return RobosuiteSimulation(scene, self.render, self.real_time, self.speed,
                                 self.use_environment, self.env_config, self.controller_config, 
                                 self.camera_view, **kwargs)


class RobosuiteSimulation(Simulation):
    ENV_OBJECTS = {"Lift": {"cube": "cube_main"}, "Stack": {"cubeA": "cubeA_main", "cubeB": "cubeB_main"}}
    CAMERA_VIEWS = {
        "frontview": 0,
        "birdview": 1, 
        "agentview": 2,
        "sideview": 3,
        "robot0_robotview": 4,
        "robot0_eye_in_hand": 5
    }
    
    def __init__(self, scene, render, real_time, speed, use_environment=None,
                 env_config=None, controller_config=None, camera_view=None, **kwargs):
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
        self.viewer = None
        
        # Mapping objects to RoboSuite IDs (instead of storing on objects)
        self.body_id_map = {}
        self.object_names = {}  # Scenic object -> RoboSuite name
        self.robots = []
        self.prev_positions = {}
        self.pending_actions = {}
        self._current_obs = None
        
        self.timestep = kwargs.get('timestep') or 0.01
        self.physics_timestep = 0.002
        self.physics_steps = int(self.timestep / self.physics_timestep)
        self.agents = []
        super().__init__(scene, **kwargs)
    
    def setup(self):
        super().setup()  # Call parent setup first to populate objects
        
        if not self.use_environment:
            raise ValueError("Only pre-configured RoboSuite environments are supported")
            
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
        
        # Prepare robots argument
        robot_arg = "Panda"  # Default robot
        if robots:
            robot_arg = [r.robot_type for r in robots] if len(robots) > 1 else robots[0].robot_type
        
        # Create RoboSuite environment
        self.robosuite_env = suite.make(self.use_environment, robots=robot_arg, **config)
        self._current_obs = self.robosuite_env.reset()
        self.model = self.robosuite_env.sim.model._model
        self.data = self.robosuite_env.sim.data._data
        
        # Set camera view
        if self.render and self.camera_view is not None:
            camera_id = self.camera_view
            if isinstance(self.camera_view, str):
                camera_id = self.CAMERA_VIEWS.get(self.camera_view.lower(), 0)
            if self.robosuite_env.viewer:
                self.robosuite_env.viewer.set_camera(camera_id=camera_id)
        
        # Store robot references
        for i, r in enumerate(robots[:len(self.robosuite_env.robots)]):
            self.robots.append(r)
            self.object_names[r] = f"robot{i}"
        
        # Apply initial positions for environment objects
        self._apply_initial_positions()
        
        # Setup body mapping
        if self.use_environment in self.ENV_OBJECTS:
            for name, body in self.ENV_OBJECTS[self.use_environment].items():
                id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
                if id != -1: 
                    self.body_id_map[name] = id
                    self.prev_positions[name] = self.data.xpos[id].copy()
        
        # Identify agents
        for obj in self.objects:
            if hasattr(obj, 'behavior') and obj.behavior:
                self.agents.append(obj)
    
    def _apply_initial_positions(self):
        """Apply initial positions for environment objects."""
        # Apply positions for objects with envObjectName
        for obj in self.objects:
            if hasattr(obj, 'envObjectName') and hasattr(obj, 'position'):
                joint_name = f"{obj.envObjectName}_joint0"
                try:
                    qpos = np.concatenate([
                        [obj.position.x, obj.position.y, obj.position.z],
                        [1, 0, 0, 0]  # identity quaternion
                    ])
                    self.robosuite_env.sim.data.set_joint_qpos(joint_name, qpos)
                except Exception as e:
                    print(f"Warning: Could not set joint position for {obj.envObjectName}: {e}")
        
        # Update physics
        self.robosuite_env.sim.forward()
        
        # Let physics settle
        for _ in range(10):
            self.robosuite_env.step(np.zeros(7))
            
        # Get fresh observations
        self._current_obs = self.robosuite_env._get_observations()
    
    def createObjectInSimulator(self, obj):
        # For pre-configured environments, objects are already created
        pass
    
    def executeActions(self, allActions):
        super().executeActions(allActions)
        self.pending_actions = {}
        
        for agent in self.agents:
            if agent in allActions and allActions[agent]:
                for action in allActions[agent]:
                    if action and hasattr(action, 'applyTo'):
                        # Store action for this agent
                        if hasattr(action, 'position_delta'):
                            self.pending_actions[agent] = ('osc', action)
                        elif hasattr(action, 'positions'):
                            self.pending_actions[agent] = ('joint', action.positions)
    
    def step(self):
        # Check if episode is done
        if hasattr(self, '_done') and self._done:
            return
            
        # Store previous positions
        for name, id in self.body_id_map.items():
            self.prev_positions[name] = self.data.xpos[id].copy()
        
        # Execute actions
        action = np.zeros(7)
        for agent, (action_type, data) in self.pending_actions.items():
            if action_type == 'osc':
                if hasattr(data, 'position_delta') and data.position_delta:
                    action[:3] = data.position_delta
                if hasattr(data, 'orientation_delta') and data.orientation_delta:
                    action[3:6] = data.orientation_delta
                if hasattr(data, 'gripper') and data.gripper is not None:
                    action[6] = data.gripper
            else:
                action = np.array(data)
            break  # RoboSuite only supports one robot action per step
        
        # Step simulation
        for _ in range(self.physics_steps):
            obs, reward, done, info = self.robosuite_env.step(action)
            self._current_obs = obs
            self._done = done
            if self.render:
                self.robosuite_env.render()
            if done:
                break
        
        self.pending_actions = {}
    
    def getProperties(self, obj, properties):
        values = {}
        
        for prop in properties:
            if prop == 'position':
                # Check if it's an environment object
                if hasattr(obj, 'envObjectName') and self._current_obs:
                    obs_key = f"{obj.envObjectName}_pos"
                    if obs_key in self._current_obs:
                        pos = self._current_obs[obs_key]
                        values[prop] = Vector(pos[0], pos[1], pos[2])
                    else:
                        values[prop] = obj.position
                else:
                    values[prop] = obj.position
                    
            elif prop == 'joint_positions' and obj in self.robots:
                # Get joint positions for robots
                robot_idx = self.robots.index(obj)
                if robot_idx < len(self.robosuite_env.robots):
                    positions = self.robosuite_env.robots[robot_idx]._joint_positions
                    values[prop] = list(positions)  # Convert numpy array to list
                else:
                    values[prop] = []
                    
            elif prop == 'eef_pos' and obj in self.robots and self._current_obs:
                pos = self._current_obs.get('robot0_eef_pos', [0, 0, 0])
                values[prop] = list(pos)  # Convert numpy array to list
                
            elif prop == 'gripper_state' and obj in self.robots and self._current_obs:
                state = self._current_obs.get('robot0_gripper_qpos', [0, 0])
                values[prop] = list(state)  # Convert numpy array to list
                
            else:
                # Default to object's own property
                values[prop] = getattr(obj, prop, None)
        
        return values
    
    def getCurrentObservation(self):
        """Public method to get current observations."""
        return self._current_obs
    
    def checkSuccess(self):
        return self.robosuite_env._check_success() if hasattr(self.robosuite_env, '_check_success') else False
    
    def destroy(self):
        if self.render and self.robosuite_env:
            print("Simulation complete. Window will close in 2 seconds...")
            time.sleep(2)
        
        try:
            if self.robosuite_env:
                self.robosuite_env.close()
                self.robosuite_env = None
        except Exception as e:
            print(f"Warning: Error closing RoboSuite environment: {e}")
        
        try:
            if self.viewer:
                self.viewer.close()
                self.viewer = None
        except Exception as e:
            print(f"Warning: Error closing viewer: {e}")