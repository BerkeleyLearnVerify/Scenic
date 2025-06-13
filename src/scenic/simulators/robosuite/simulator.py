# src/scenic/simulators/robosuite/simulator.py
"""RoboSuite Simulator - Fixed shape handling."""

import numpy as np
import mujoco
from typing import Dict, List, Any, Optional, Tuple

try:
    import robosuite as suite
    from robosuite.models import MujocoWorldBase
    from robosuite.models.arenas import TableArena, EmptyArena
    from robosuite.models.objects import BoxObject, BallObject, CylinderObject
except ImportError as e:
    suite = None
    _import_error = e

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector


class RobosuiteSimulator(Simulator):
    def __init__(self, use_table=False, **kwargs):
        super().__init__()
        
        if suite is None:
            raise RuntimeError(f"Unable to import RoboSuite: {_import_error}")
        
        self.render_mode = kwargs.get('has_renderer', True)
        self.use_table = use_table
        
    def createSimulation(self, scene, **kwargs):
        return RobosuiteSimulation(scene, self.render_mode, self.use_table, **kwargs)


class RobosuiteSimulation(Simulation):
    def __init__(self, scene, render_mode=True, use_table=False, **kwargs):
        self.render_mode = render_mode
        self.use_table = use_table
        self.model = None
        self.data = None
        self.viewer = None
        
        # Build custom world with proper shape handling
        self.world = self._build_custom_world(scene)
        
        timestep = 0.01
        kwargs.setdefault('maxSteps', 1000)
        kwargs.setdefault('name', 'Custom-Scenic-World')
        
        super().__init__(scene, timestep=timestep, **kwargs)
        
    def _build_custom_world(self, scene):
        """Build custom world with PROPER SHAPE HANDLING."""
        print(f"Building custom world with {len(scene.objects)} Scenic objects...")
        
        # Start with empty world with ground
        world = MujocoWorldBase()
        
        if self.use_table:
            arena = TableArena()
            arena.set_origin([0, 0, 0])
            world.merge(arena)
            print("Added table with ground")
        else:
            arena = EmptyArena()
            world.merge(arena) 
            print("Added ground plane only")
        
        # Add Scenic objects with proper shape detection
        for i, obj in enumerate(scene.objects):
            print(f"  Adding {type(obj).__name__} at {obj.position}")
            
            # Get object properties
            width = getattr(obj, 'width', 0.05)
            length = getattr(obj, 'length', 0.05) 
            height = getattr(obj, 'height', 0.05)
            color = getattr(obj, 'color', (0.5, 0.5, 0.5))
            
            # Convert color
            if hasattr(color, '__iter__') and len(color) >= 3:
                rgba = [color[0], color[1], color[2], 1.0]
            else:
                rgba = [0.5, 0.5, 0.5, 1.0]
            
            # Determine shape based on CLASS NAME (not shape property)
            class_name = type(obj).__name__.lower()
            
            if 'ball' in class_name:
                # Create ball/sphere
                radius = width / 2
                robosuite_obj = BallObject(
                    name=f"scenic_ball_{i}",
                    size=[radius],
                    rgba=rgba
                ).get_obj()
                print(f"    → Created BALL (radius: {radius:.3f})")
                
            elif 'cylinder' in class_name:
                # Create cylinder
                radius = width / 2
                half_height = height / 2
                robosuite_obj = CylinderObject(
                    name=f"scenic_cylinder_{i}",
                    size=[radius, half_height],
                    rgba=rgba
                ).get_obj()
                print(f"    → Created CYLINDER (radius: {radius:.3f}, height: {height:.3f})")
                
            else:
                # Default to box/cube
                robosuite_obj = BoxObject(
                    name=f"scenic_cube_{i}",
                    size=[width/2, length/2, height/2],
                    rgba=rgba
                ).get_obj()
                print(f"    → Created CUBE (size: {width:.3f}×{length:.3f}×{height:.3f})")
            
            # Position object
            pos_x = obj.position.x
            pos_y = obj.position.y  
            pos_z = obj.position.z
            robosuite_obj.set('pos', f'{pos_x} {pos_y} {pos_z}')
            
            # Add to world
            world.worldbody.append(robosuite_obj)
            print(f" Positioned at ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})")
        
        print("Custom world with PROPER SHAPES built successfully")
        return world
        
    def setup(self):
        super().setup()
        
        # Create MuJoCo model
        self.model = self.world.get_model(mode="mujoco")
        self.data = mujoco.MjData(self.model)
        
        print(f" MuJoCo model created:")
        print(f" Bodies: {self.model.nbody}")
        print(f" Geoms: {self.model.ngeom}")
        
        # Create viewer
        if self.render_mode:
            try:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                print(" Viewer launched")
            except Exception as e:
                print(f" Viewer error: {e}")
                self.viewer = None
        
    def createObjectInSimulator(self, obj):
        return True
        
    def getProperties(self, obj, properties):
        values = {}
        for prop in properties:
            values[prop] = getattr(obj, prop, None)
        return values
        
    def step(self):
        if self.model and self.data:
            mujoco.mj_step(self.model, self.data)
            if self.viewer:
                self.viewer.sync()
                
    def render(self):
        pass
            
    def destroy(self):
        if self.viewer:
            try:
                self.viewer.close()
            except:
                pass
            self.viewer = None
