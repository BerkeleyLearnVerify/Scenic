# src/scenic/simulators/robosuite/simulator.py
"""RoboSuite Simulator for Scenic."""

import numpy as np
import mujoco

try:
    import robosuite as suite
    from robosuite.models import MujocoWorldBase
    from robosuite.models.arenas import EmptyArena
    from robosuite.models.objects import BoxObject, BallObject, CylinderObject
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
        
        # Forward dynamics to initialize
        mujoco.mj_forward(self.model, self.data)
        
        # Initialize viewer
        if self.render:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            except:
                self.viewer = None
    
    def createObjectInSimulator(self, obj):
        """Create a Scenic object in the RoboSuite simulator."""
        self._create_object(obj)
    
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
    
    def getProperties(self, obj, properties):
        """Read object properties from the simulator."""
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
        
    def destroy(self):
        """Clean up the simulation."""
        if self.viewer:
            try:
                self.viewer.close()
            except:
                pass
            self.viewer = None