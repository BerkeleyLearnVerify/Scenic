# src/scenic/simulators/robosuite/simulator.py
"""RoboSuite Simulator for Scenic - Version 3.0 (Reduced)"""

import numpy as np
import mujoco
import time
import math
from typing import Dict, List, Any

try:        
    import robosuite as suite
    from robosuite.models import MujocoWorldBase
    from robosuite.models.arenas import EmptyArena, TableArena
    from robosuite.models.objects import BoxObject, BallObject, CylinderObject
    from robosuite.robots import ROBOT_CLASS_MAPPING
except ImportError as e:
    suite = None
    _import_error = e  # why not throw error early (s)

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector
from .utils import scenic_to_rgba # why only one module from .utils, that means other are redundent (s)


class RobosuiteSimulator(Simulator):
    def __init__(self, render=True, real_time=True, speed=1.0, use_environment=None, 
                 env_config=None, controller_config=None):
        super().__init__()
        if suite is None:
            raise RuntimeError(f"Unable to import RoboSuite: {_import_error}")
        self.render, self.real_time, self.speed = render, real_time, speed
        self.use_environment = use_environment
        self.env_config = env_config or {}
        self.controller_config = controller_config
        
    def createSimulation(self, scene, **kwargs):
        return RobosuiteSimulation(scene, self.render, self.real_time, self.speed,
                                 self.use_environment, self.env_config, self.controller_config, **kwargs)


class RobosuiteSimulation(Simulation):
    ENV_OBJECTS = {"Lift": {"cube": "cube_main"}, "Stack": {"cubeA": "cubeA_main", "cubeB": "cubeB_main"}}
    
    def __init__(self, scene, render, real_time, speed, use_environment=None,
                 env_config=None, controller_config=None, **kwargs):
        self.render, self.real_time, self.speed = render, real_time, speed
        self.use_environment, self.env_config = use_environment, env_config or {}
        self.controller_config = controller_config
        self.robosuite_env = self.world = self.model = self.data = self.viewer = None
        self._body_id_map, self._prev_positions, self._robots = {}, {}, []
        self._current_obs, self._pending_actions = None, {}
        self.timestep = kwargs.get('timestep') or 0.01
        self.physics_timestep = 0.002
        self.physics_steps = int(self.timestep / self.physics_timestep)
        self.agents = []
        super().__init__(scene, **kwargs)
    
    def setup(self):
        if self.use_environment:
            # Pre-configured environment
            robots = [obj for obj in self.scene.objects if hasattr(obj, 'robot_type')]
            config = {**self.env_config, 'has_renderer': self.render, 'render_camera': "frontview",
                     'camera_names': ["frontview", "robot0_eye_in_hand"], 'controller_configs': self.controller_config}
            if robots:
                config['robots'] = [r.robot_type for r in robots] if len(robots) > 1 else robots[0].robot_type
            
            self.robosuite_env = suite.make(self.use_environment, **config)
            self._current_obs = self.robosuite_env.reset()
            self.model = self.robosuite_env.sim.model._model
            self.data = self.robosuite_env.sim.data._data
            
            for i, r in enumerate(robots[:len(self.robosuite_env.robots)]):
                r._robosuite_robot = self.robosuite_env.robots[i]
                r._robot_model = r._robosuite_robot.robot_model
                self._robots.append(r)
            
            # Modify env objects
            for obj in self.scene.objects:
                if hasattr(obj, '_env_object_name') and hasattr(obj, 'position'):
                    env_obj = getattr(self.robosuite_env, obj._env_object_name, None)
                    if env_obj and hasattr(env_obj, 'get_obj'):
                        body = env_obj.get_obj().find(f".//body[@name='{obj._env_object_name}_main']")
                        if body is not None:
                            body.set('pos', f"{obj.position.x} {obj.position.y} {obj.position.z}")
            
            # Add custom objects
            world = self.robosuite_env.model
            for obj in self.scene.objects:
                if not any([hasattr(obj, attr) for attr in ['robot_type', '_env_object_name']]) and \
                   type(obj).__name__ not in ['Arena', 'EmptyArena', 'TableArena']:
                    self._add_to_env(world, obj)
            
            self._current_obs = self.robosuite_env.reset()
        else:
            # Custom world
            self.world = MujocoWorldBase()
            from .xml_builder import RoboSuiteXMLBuilder
            self.xml_builder = RoboSuiteXMLBuilder()
            
            arena = TableArena() if any(type(o).__name__ == 'Table' for o in self.scene.objects) else EmptyArena()
            self.world.merge(arena)
            super().setup()
            
            self.model = self.world.get_model(mode="mujoco")
            self.data = mujoco.MjData(self.model)
            self.model.opt.gravity[2] = -9.81
            self.model.opt.timestep = self.physics_timestep
            
            if self.render:
                try: self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                except: self.viewer = None
        
        # Setup body mapping
        if self.use_environment in self.ENV_OBJECTS:
            for name, body in self.ENV_OBJECTS[self.use_environment].items():
                id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
                if id != -1: self._body_id_map[name] = id
        
        for obj in self.objects:
            self._map_body(obj)
        
        # Initialize robots
        for r in self._robots:
            if hasattr(r, '_robot_model') and hasattr(r, '_initial_qpos'):
                for joint, qpos in zip(r._robot_model.joints, r._initial_qpos):
                    jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                    if jid != -1: self.data.qpos[self.model.jnt_qposadr[jid]] = qpos
        
        if not self.robosuite_env:
            mujoco.mj_forward(self.model, self.data)
        
        # Configure dynamics
        for obj in self.scene.objects:
            if hasattr(obj, 'robot_type'):
                obj._dynamicProperties.update({'joint_positions': True, 'eef_pos': True, 'gripper_state': True})
                if hasattr(obj, 'behavior') and obj.behavior:
                    self.agents.append(obj)
            else:
                obj._dynamicProperties['position'] = True
    
    def _map_body(self, obj):
        id = -1
        if hasattr(obj, '_robosuite_name'):
            for suffix in ["_main", "", "_body"]:
                id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, obj._robosuite_name + suffix)
                if id != -1:
                    self._body_id_map[obj._robosuite_name] = id
                    break
        elif hasattr(obj, '_robot_model') or hasattr(obj, '_robosuite_robot'):
            name = (obj._robosuite_robot.robot_model.naming_prefix if hasattr(obj, '_robosuite_robot') 
                   else obj._robot_model.naming_prefix)
            id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name + "base_link")
            if id != -1: self._body_id_map[name] = id
        
        if id != -1:
            key = obj._robosuite_name if hasattr(obj, '_robosuite_name') else name
            self._prev_positions[key] = self.data.xpos[id].copy()
    
    def _add_to_env(self, world, obj):
        cls = type(obj).__name__.lower()
        name = f"custom_{id(obj)}"
        
        # Create object
        sizes = {'ball': [obj.width/2], 'cylinder': [obj.width/2, obj.height/2]}
        size = sizes.get(cls.split('_')[0], [obj.width/2, obj.length/2, obj.height/2])
        
        ObjClass = {'ball': BallObject, 'cylinder': CylinderObject}.get(cls.split('_')[0], BoxObject)
        rs_obj = ObjClass(name, size=size, density=getattr(obj, 'density', 1000), 
                         rgba=scenic_to_rgba(getattr(obj, 'color', (0.5, 0.5, 0.5))))
        
        # Position
        z = obj.height/2 if hasattr(obj, 'height') else 0
        body = rs_obj.get_obj().find(f".//body[@name='{name}_main']") or rs_obj.get_obj().find(f".//body[@name='{name}']")
        if body is not None:
            body.set('pos', f'{obj.position.x} {obj.position.y} {obj.position.z + z}')
        
        world.merge(rs_obj)
        obj._robosuite_name = name
    
    def createObjectInSimulator(self, obj):
        if self.robosuite_env: return
        
        obj_type = type(obj).__name__
        if obj_type in ['Arena', 'EmptyArena', 'TableArena', 'BinsArena', 'PegsArena', 'WipeArena', 'CustomArena', 'Table']:
            return
        
        if hasattr(obj, 'robot_type'):
            robot_class = ROBOT_CLASS_MAPPING.get(obj.robot_type)
            if not robot_class: raise ValueError(f"Unknown robot type: {obj.robot_type}")
            
            robot = robot_class(robot_type=obj.robot_type, idn=len(self._robots), 
                               initial_qpos=getattr(obj, 'initial_qpos', None), control_freq=20)
            robot.load_model()
            robot.robot_model.set_base_xpos([obj.position.x, obj.position.y, obj.position.z])
            if hasattr(obj, 'yaw'): robot.robot_model.set_base_ori([0, 0, obj.yaw])
            
            self.world.merge(robot.robot_model)
            obj._robot = robot
            obj._robot_model = robot.robot_model
            obj._initial_qpos = getattr(obj, 'initial_qpos', None) or robot.init_qpos
            self._robots.append(obj)
        elif obj_type == 'PositionableTable':
            import xml.etree.ElementTree as ET
            name = f"table_{id(obj)}"
            h, w, l = obj.height, obj.width/2, obj.length/2
            
            legs = ""
            for i, (x, y) in enumerate([(w-0.1, l-0.1), (-w+0.1, l-0.1), (w-0.1, -l+0.1), (-w+0.1, -l+0.1)]):
                legs += f'<geom name="{name}_leg{i+1}" type="cylinder" pos="{x} {y} {(h-0.05)/2}" size="0.025 {(h-0.05)/2}" rgba="0.5 0.5 0.5 1"/>'
            
            xml = f'''<?xml version="1.0"?><mujoco model="{name}"><worldbody>
            <body name="{name}_main" pos="{obj.position.x} {obj.position.y} {obj.position.z}">
                <geom name="{name}_top" type="box" pos="0 0 {h-0.025}" size="{w} {l} 0.025" friction="1 0.005 0.0001" rgba="0.9 0.9 0.9 1"/>
                <geom name="{name}_top_visual" type="box" pos="0 0 {h-0.025}" size="{w} {l} 0.025" rgba="0.95 0.95 0.95 1" conaffinity="0" contype="0" group="1"/>
                {legs}
            </body></worldbody></mujoco>'''
            
            self.world.worldbody.append(ET.fromstring(xml).find(".//body"))
            obj._robosuite_name = name
        elif hasattr(obj, 'xml_path') or hasattr(obj, 'xml_string'):
            rs_obj = self.xml_builder.create_scenic_object(obj)
            z = obj.height/2 if hasattr(obj, 'height') else 0
            rs_obj.set_base_xpos([obj.position.x, obj.position.y, obj.position.z + z])
            if hasattr(obj, 'yaw'): rs_obj.set_base_ori([0, 0, obj.yaw])
            self.world.merge(rs_obj)
            obj._robosuite_name = rs_obj.name
        else:
            self._create_geom(obj)
    
    def _create_geom(self, obj):
        cls = type(obj).__name__.lower()
        name = f"obj_{id(obj)}"
        physics = {'density': getattr(obj, 'density', 1000), 'friction': getattr(obj, 'friction', (1.0, 0.005, 0.0001)),
                  'rgba': scenic_to_rgba(getattr(obj, 'color', (0.5, 0.5, 0.5)))}
        
        if 'ball' in cls:
            rs_obj, z = BallObject(name, size=[obj.width/2], **physics), obj.width/2
        elif 'cylinder' in cls:
            rs_obj, z = CylinderObject(name, size=[obj.width/2, obj.height/2], **physics), obj.height/2
        else:
            rs_obj, z = BoxObject(name, size=[obj.width/2, obj.length/2, obj.height/2], **physics), obj.height/2
        
        rs_obj.get_obj().set('pos', f'{obj.position.x} {obj.position.y} {obj.position.z + z}')
        self.world.worldbody.append(rs_obj.get_obj())
        obj._robosuite_name = name
    
    def executeActions(self, allActions):
        super().executeActions(allActions)
        self._pending_actions = {}
        for r in self._robots:
            if r in allActions and allActions[r]:
                for action in allActions[r]:
                    if action:
                        if action.__class__.__name__ == 'SetJointPositions':
                            self._pending_actions[r] = ('joint', action.positions)
                        elif action.__class__.__name__ == 'OSCPositionAction':
                            self._pending_actions[r] = ('osc', action)
    
    def step(self):
        for name, id in self._body_id_map.items():
            self._prev_positions[name] = self.data.xpos[id].copy()
        
        if self.robosuite_env:
            action = np.zeros(7)
            for r, (t, d) in self._pending_actions.items():
                if t == 'osc':
                    if d.position_delta: action[:3] = d.position_delta
                    if d.orientation_delta: action[3:6] = d.orientation_delta
                    if d.gripper is not None: action[6] = d.gripper
                else:
                    action = np.array(d)
                break
            
            for _ in range(self.physics_steps):
                self._current_obs, _, _, _ = self.robosuite_env.step(action)
                if self.render: self.robosuite_env.render()
        else:
            # Apply actions
            for r, (t, d) in self._pending_actions.items():
                if t == 'joint':
                    for i, act in enumerate(r._robot_model.actuators[:len(d)]):
                        aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act)
                        if aid != -1: self.data.ctrl[aid] = float(d[i])
            
            # Step
            start = None
            for i in range(self.physics_steps):
                mujoco.mj_step(self.model, self.data)
                if self.real_time and self.viewer and i == 0: start = time.monotonic()
                if self.viewer and i % 25 == 0: self.viewer.sync()
            
            if self.real_time and self.viewer and start:
                time.sleep(max(0, self.timestep / self.speed - (time.monotonic() - start)))
            
            mujoco.mj_forward(self.model, self.data)
        
        self._pending_actions = {}
        
        # Update robots
        for r in self._robots:
            if hasattr(r, '_robosuite_robot'):
                r.joint_positions = r._robosuite_robot._joint_positions
                if self._current_obs:
                    r.eef_pos = self._current_obs.get('robot0_eef_pos', [0, 0, 0])
                    r.gripper_state = self._current_obs.get('robot0_gripper_qpos', [0, 0])
            else:
                r.joint_positions = [self.data.qpos[self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)]] 
                                   for j in r._robot_model.joints if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) != -1]
    
    def getProperties(self, obj, properties):
        values = {}
        
        # Get body ID
        id = -1
        if hasattr(obj, '_robosuite_name'):
            id = self._body_id_map.get(obj._robosuite_name, -1)
        elif hasattr(obj, '_robot_model') or hasattr(obj, '_robosuite_robot'):
            name = (obj._robosuite_robot.robot_model.naming_prefix if hasattr(obj, '_robosuite_robot') 
                   else obj._robot_model.naming_prefix)
            id = self._body_id_map.get(name, -1)
        elif hasattr(obj, '_env_object_name'):
            id = self._body_id_map.get(obj._env_object_name, -1)
        
        for prop in properties:
            if prop == 'position':
                if id != -1:
                    pos = self.data.xpos[id]
                    z_off = obj.height/2 if hasattr(obj, 'height') and not hasattr(obj, 'robot_type') and \
                           any(s in type(obj).__name__.lower() for s in ['cube', 'box', 'ball', 'cylinder']) else 0
                    values[prop] = Vector(pos[0], pos[1], pos[2] - z_off)
                elif hasattr(obj, '_env_object_name') and self._current_obs and f"{obj._env_object_name}_pos" in self._current_obs:
                    pos = self._current_obs[f"{obj._env_object_name}_pos"]
                    values[prop] = Vector(pos[0], pos[1], pos[2])
                else:
                    values[prop] = getattr(obj, prop)
            elif prop == 'velocity' and id != -1:
                if self.model.body_dofnum[id] >= 3:
                    v = self.data.qvel[self.model.body_dofadr[id]:self.model.body_dofadr[id]+3]
                else:
                    name = obj._robosuite_name if hasattr(obj, '_robosuite_name') else ''
                    v = (self.data.xpos[id] - self._prev_positions.get(name, self.data.xpos[id])) / self.timestep
                values[prop] = Vector(v[0], v[1], v[2])
            elif prop == 'speed' and id != -1:
                v = values.get('velocity') or self.getProperties(obj, ['velocity']).get('velocity', Vector(0,0,0))
                values[prop] = np.linalg.norm([v.x, v.y, v.z])
            elif prop in ['yaw', 'pitch', 'roll'] and id != -1:
                w, x, y, z = self.data.xquat[id]
                if prop == 'yaw': values[prop] = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                elif prop == 'pitch': values[prop] = math.asin(np.clip(2*(w*y-z*x), -1, 1))
                elif prop == 'roll': values[prop] = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
            else:
                values[prop] = getattr(obj, prop)
        
        return values
    
    def checkSuccess(self):
        return self.robosuite_env._check_success() if self.robosuite_env and hasattr(self.robosuite_env, '_check_success') else False
    
    def destroy(self):
        if self.render and (self.robosuite_env or self.viewer):
            print("Simulation complete. Window will close in 2 seconds...")
            time.sleep(2)
        
        if self.robosuite_env:
            self.robosuite_env.close()
            self.robosuite_env = None
        
        if self.viewer:
            try: self.viewer.close()
            except: pass
            self.viewer = None