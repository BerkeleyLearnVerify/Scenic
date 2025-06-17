# src/scenic/simulators/robosuite/simulator.py
"""RoboSuite Simulator for Scenic."""

import numpy as np
import mujoco

try:
    import robosuite as suite
    from robosuite.models import MujocoWorldBase
    from robosuite.models.arenas import EmptyArena
    from robosuite.models.objects import BoxObject, BallObject, CylinderObject
    from robosuite.robots import ROBOT_CLASS_MAPPING
except ImportError as e:
    suite = None
    _import_error = e

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector
from .utils import scenic_to_rgba


class RobosuiteSimulator(Simulator):
    """Simulator interface for RoboSuite."""
    
    def __init__(self, render=True, real_time=True, speed=1.0):
        super().__init__()
        if suite is None:
            raise RuntimeError(f"Unable to import RoboSuite: {_import_error}")
        self.render = render
        self.real_time = real_time
        self.speed = speed
        
    def createSimulation(self, scene, **kwargs):
        return RobosuiteSimulation(scene, self.render, self.real_time, self.speed, **kwargs)


class RobosuiteSimulation(Simulation):
    """A simulation running in RoboSuite."""
    
    def __init__(self, scene, render, real_time, speed, **kwargs):
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.world = None
        self.model = None
        self.data = None
        self.viewer = None
        self._step_count = 0
        self._body_id_map = {}
        self._prev_positions = {}
        self._robots = []
        
        # Set timestep parameters
        self.timestep = kwargs.get('timestep') or 0.1
        self.physics_timestep = 0.002
        self.physics_steps = int(self.timestep / self.physics_timestep)
        
        super().__init__(scene, **kwargs)
        
    def setup(self):
        """Set up the RoboSuite simulation."""
        # Create world with empty arena
        self.world = MujocoWorldBase()
        arena = EmptyArena()
        self.world.merge(arena)
        
        # Create objects
        super().setup()
        
        # Build MuJoCo model
        self.model = self.world.get_model(mode="mujoco")
        self.data = mujoco.MjData(self.model)
        
        # Map body names to IDs
        self._setup_body_mapping()
        
        # Initialize robot positions
        self._initialize_robots()
        
        # Forward dynamics to initialize
        mujoco.mj_forward(self.model, self.data)
        
        # Mark joint_positions as dynamic for all robots
        for robot_obj in self._robots:
            robot_obj._dynamicProperties['joint_positions'] = True
        
        # Initialize viewer
        if self.render:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            except:
                self.viewer = None
    
    def _setup_body_mapping(self):
        """Map body names to MuJoCo IDs."""
        for obj in self.objects:
            if hasattr(obj, '_robosuite_name'):
                # Try different body name patterns
                body_names = [
                    obj._robosuite_name + "_main",
                    obj._robosuite_name,
                    obj._robosuite_name + "_body"
                ]
                
                for body_name in body_names:
                    body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
                    if body_id != -1:
                        self._body_id_map[obj._robosuite_name] = body_id
                        self._prev_positions[obj._robosuite_name] = self.data.xpos[body_id].copy()
                        break
                        
            elif hasattr(obj, '_robot_model'):
                robot_name = obj._robot_model.naming_prefix
                base_body_name = robot_name + "base_link"
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, base_body_name)
                if body_id != -1:
                    self._body_id_map[robot_name] = body_id
                    self._prev_positions[robot_name] = self.data.xpos[body_id].copy()
    
    def _initialize_robots(self):
        """Initialize robot joint positions."""
        for robot_obj in self._robots:
            robot_model = robot_obj._robot_model
            if hasattr(robot_obj, '_initial_qpos') and robot_obj._initial_qpos:
                init_qpos = robot_obj._initial_qpos
                
                # Set joint positions
                for joint_name, qpos_value in zip(robot_model.joints, init_qpos):
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    if joint_id != -1:
                        qpos_addr = self.model.jnt_qposadr[joint_id]
                        self.data.qpos[qpos_addr] = qpos_value
                
                # Initialize actuators
                for i, actuator_name in enumerate(robot_model.actuators):
                    if i < len(init_qpos):
                        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                        if actuator_id != -1:
                            self.data.ctrl[actuator_id] = init_qpos[i]
    
    def createObjectInSimulator(self, obj):
        """Create a Scenic object in the RoboSuite simulator."""
        if hasattr(obj, 'robot_type'):
            self._create_robot(obj)
        else:
            self._create_object(obj)
    
    def _create_robot(self, obj):
        """Create a robot in the simulation."""
        robot_type = getattr(obj, 'robot_type', 'Panda')
        initial_qpos = getattr(obj, 'initial_qpos', None)
        
        # Create robot instance
        robot_class = ROBOT_CLASS_MAPPING.get(robot_type)
        if not robot_class:
            raise ValueError(f"Unknown robot type: {robot_type}")
        
        robot = robot_class(
            robot_type=robot_type,
            idn=len(self._robots),
            initial_qpos=initial_qpos,
            control_freq=20,
        )
        
        robot.load_model()
        robot_model = robot.robot_model
        
        # Set position and orientation
        pos = obj.position
        robot_model.set_base_xpos([pos.x, pos.y, pos.z])
        if hasattr(obj, 'yaw'):
            robot_model.set_base_ori([0, 0, obj.yaw])
        
        # Merge into world
        self.world.merge(robot_model)
        
        # Store references
        obj._robot = robot
        obj._robot_model = robot_model
        obj._robot_type = robot_type
        obj._initial_qpos = initial_qpos or robot.init_qpos
        self._robots.append(obj)
    
    def _create_object(self, obj):
        """Create a regular object in the simulation."""
        class_name = type(obj).__name__.lower()
        name = f"obj_{len(self.world.worldbody)}"
        rgba = scenic_to_rgba(getattr(obj, 'color', (0.5, 0.5, 0.5)))
        
        # Common physics parameters
        physics_params = {
            'density': getattr(obj, 'density', 1000),
            'solref': getattr(obj, 'solref', (0.02, 1.0)),
            'solimp': getattr(obj, 'solimp', (0.9, 0.95, 0.001, 0.5, 2.0)),
            'friction': getattr(obj, 'friction', (1.0, 0.005, 0.0001)),
            'rgba': rgba
        }
        
        # Create object based on type
        if 'ball' in class_name:
            rs_obj = BallObject(name, size=[obj.width/2], **physics_params)
            center_z = obj.position.z + obj.width/2  # Ball radius
        elif 'cylinder' in class_name:
            rs_obj = CylinderObject(name, size=[obj.width/2, obj.height/2], **physics_params)
            center_z = obj.position.z + obj.height/2  # Half cylinder height
        else:  # Default to box
            rs_obj = BoxObject(name, size=[obj.width/2, obj.length/2, obj.height/2], **physics_params)
            center_z = obj.position.z + obj.height/2  # Half box height
        
        # Position object - Scenic position is bottom center, adjust for MuJoCo center
        mj_obj = rs_obj.get_obj()
        pos = obj.position
        mj_obj.set('pos', f'{pos.x} {pos.y} {center_z}')
        
        self.world.worldbody.append(mj_obj)
        obj._robosuite_name = name
    
    def executeActions(self, allActions):
        """Execute actions for all agents."""
        super().executeActions(allActions)
        
        for robot_obj in self._robots:
            if robot_obj in allActions and allActions[robot_obj]:
                for action in allActions[robot_obj]:
                    if action and action.__class__.__name__ == 'SetJointPositions':
                        self._apply_robot_action(robot_obj, action.positions)
    
    def _apply_robot_action(self, robot_obj, positions):
        """Apply joint position control to a robot."""
        robot_model = robot_obj._robot_model
        
        if not isinstance(positions, (list, tuple)):
            return
            
        # Apply joint position control
        for i, actuator in enumerate(robot_model.actuators):
            if i < len(positions):
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
                if actuator_id != -1:
                    self.data.ctrl[actuator_id] = float(positions[i])
    
    def getProperties(self, obj, properties):
        """Read object properties from the simulator."""
        if hasattr(obj, '_robot_model'):
            return self._get_robot_properties(obj, properties)
        
        if not hasattr(obj, '_robosuite_name'):
            return {prop: getattr(obj, prop) for prop in properties}
        
        values = {}
        for prop in properties:
            if prop == 'position':
                # For now, return the original position
                values[prop] = obj.position
            else:
                values[prop] = getattr(obj, prop)
                
        return values
    
    def _get_robot_properties(self, obj, properties):
        """Get properties for a robot object."""
        robot_prefix = obj._robot_model.naming_prefix
        body_id = self._body_id_map.get(robot_prefix, -1)
        
        if body_id == -1:
            return {prop: getattr(obj, prop) for prop in properties}
        
        values = {}
        for prop in properties:
            if prop == 'position':
                pos = self.data.xpos[body_id]
                values[prop] = Vector(pos[0], pos[1], pos[2])
            elif prop == 'joint_positions':
                joint_pos = []
                for joint in obj._robot_model.joints:
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                    if joint_id != -1:
                        qpos_addr = self.model.jnt_qposadr[joint_id]
                        joint_pos.append(self.data.qpos[qpos_addr])
                values[prop] = joint_pos
                # Update object's attribute for record statement
                obj.joint_positions = joint_pos
            else:
                values[prop] = getattr(obj, prop)
        
        return values
        
    def destroy(self):
        """Clean up the simulation."""
        if self.viewer:
            try:
                self.viewer.close()
            except:
                pass
            self.viewer = None