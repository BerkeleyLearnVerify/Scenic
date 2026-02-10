import math
from collections.abc import Callable
from typing import List

from scenic.core.object_types import Object

from scenic.core.distributions import distributionFunction

import dm_control 
from dm_control import mjcf
import numpy as np

import mujoco

class MujocoBody(Object):
    semanticColor: None

    """Abstract class for Mujoco objects."""
    def __init__(self, properties, *args, **kwargs):
        super().__init__(properties, *args, **kwargs)
        self.body_name = None
        # self.semanticColor = None
    
    def semanticColorFromGeom(self, geom_name):
        """Override to provide per-geom colors for segmentation."""
        return None

    def model(self):
        return None

class DynamicMujocoBody(MujocoBody):
    """Dynamic Mujoco Body"""
    def __init__(self, properties, *args, **kwargs):
        super().__init__(properties, *args, **kwargs)

    def control(self, model, data):
        """Apply control inputs (forces/torques) to the simulation."""
        raise NotImplementedError("Error: control not implemented for object")

class Terrain(MujocoBody):
    """
    Abstract class for objects added together to make a Ground.

    This is not rendered as a separate MuJoCo body since it doesn't actually 
    correspond to a MuJoCo body. Only the overall Ground has a body with hfield geometry.
    """
    
    allowCollisions: True
    render: False  # Terrain objects are not rendered separately.

    def heightAt(self, pt):
        """Get height at a given point."""
        offset = pt - self.position
        return self.heightAtOffset(offset)

    def heightAtOffset(self, offset):
        """Get height at offset from terrain center. Must be implemented by subclasses."""
        raise NotImplementedError('should be implemented by subclasses')
    
    def get_mujoco_xml(self, obj_counter, position, quaternion):
        """Terrain objects don't generate XML - they only contribute to Ground."""
        return {
            'body': '',
            'assets': '',
            'actuators': '',
            'sensors': ''
        }

class Hill(Terrain):
    """
    Terrain shaped like a Gaussian.

    Attributes:
        height (float): height of the hill (default 1).
        spread (float): standard deviation as a fraction of the hill's size
            (default 0.25).
    """

    height: 1
    spread: 0.25

    def heightAtOffset(self, offset):
        """Calculate Gaussian height at offset from hill center."""
        dx, dy, _ = offset
        if not (-self.hw < dx < self.hw and -self.hl < dy < self.hl):
            return 0
        sx, sy = dx / (self.width * self.spread), dy / (self.length * self.spread)
        nh = math.exp(-((sx * sx) + (sy * sy)) * 0.5)
        return self.height * nh


class Ground(MujocoBody):
    """
    Ground surface with irregular terrain using MuJoCo heightfield.

    Implemented using MuJoCo's hfield asset type.

    Attributes:
        allowCollisions (bool): default value True (overriding default from Object).
        width (float): width of the ground in meters (default 10).
        length (float): length of the ground in meters (default 10).
        gridSize (int): resolution of the heightfield grid (default 20).
        gridSizeX (int): X resolution (defaults to gridSize).
        gridSizeY (int): Y resolution (defaults to gridSize).
        baseThickness (float): thickness of the base below lowest terrain point (default 0.1).
        terrain (tuple): tuple of Terrain objects to combine (default empty).
        heights (tuple): computed height values for the grid.
    """

    allowCollisions: True
    
    width: 10
    length: 10
    
    gridSize: 20
    gridSizeX: self.gridSize
    gridSizeY: self.gridSize
    baseThickness: 0.1
    terrain: ()
    heights: Ground.heightsFromTerrain(self.terrain, self.gridSizeX, self.gridSizeY,
                                       self.width, self.length)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.instance_id = None
        self.semanticColor = [128, 128, 128]  # Gray for ground.

    @staticmethod
    @distributionFunction
    def heightsFromTerrain(terrain, gridSizeX, gridSizeY, width, length):
        """Generate height data from terrain objects."""
        for elem in terrain:
            if not isinstance(elem, Terrain):
                raise RuntimeError(f'Ground terrain element {elem} is not a Terrain')
        
        heights = []
        if gridSizeX < 2 or gridSizeY < 2:
            raise RuntimeError(f'invalid grid size {gridSizeX} x {gridSizeY} for Ground')
        
        dx, dy = width / (gridSizeX - 1), length / (gridSizeY - 1)
        
        # Sample grid points from -length/2 to +length/2 and -width/2 to +width/2.
        y = -length / 2
        for i in range(gridSizeY):
            row = []
            x = -width / 2
            for j in range(gridSizeX):
                # Create a Scenic Vector for the grid point.
                from scenic.core.vectors import Vector
                pt = Vector(x, y, 0)
                
                # Sum heights from all terrain elements at this point.
                height = sum(elem.heightAt(pt) for elem in terrain)
                row.append(height)
                x += dx
            heights.append(tuple(row))
            y += dy
        return tuple(heights)

    def get_mujoco_xml(self, obj_counter, position, quaternion):
        """Generate MuJoCo XML for heightfield ground."""
        u_id = f"ground_{obj_counter}"
        self.instance_id = u_id
        self.body_name = f"frame_{u_id}"
        
        # Convert heights to flat array for MuJoCo.
        # MuJoCo expects heights in row-major order.
        heights_flat = []
        for row in self.heights:
            heights_flat.extend(row)
        
        # Convert to numpy array for statistics.
        heights_array = np.array(heights_flat)
        min_height = float(np.min(heights_array))
        max_height = float(np.max(heights_array))
        height_range = max_height - min_height
        
        # If terrain is flat, use a small range to avoid issues.
        if height_range < 0.001:
            height_range = 0.001
        
        # Normalize heights to [0, 1] range for MuJoCo.
        normalized_heights = (heights_array - min_height) / height_range
        
        # Create height data string for XML.
        height_data_str = ' '.join(f'{h:.6f}' for h in normalized_heights)
        
        # MuJoCo hfield size: [size_x, size_y, elevation_z, base_z]
        # size_x, size_y: half-sizes in x and y
        # elevation_z: maximum elevation (scaled by height data)
        # base_z: depth of the base below the lowest point
        size_x = self.width / 2
        size_y = self.length / 2
        elevation_z = height_range
        base_z = self.baseThickness
        
        # Return as dictionary with separate sections.
        asset_xml = f'''        <hfield name="terrain_{u_id}" 
                nrow="{self.gridSizeY}" 
                ncol="{self.gridSizeX}" 
                size="{size_x} {size_y} {elevation_z} {base_z}"
                elevation="{height_data_str}"/>'''
        
        body_xml = f'''    <body name="{self.body_name}" pos="{position}" quat="{quaternion}">
        <geom name="ground_geom_{u_id}" 
              type="hfield" 
              hfield="terrain_{u_id}"
              rgba="0.55 0.45 0.35 1"
              contype="1" 
              conaffinity="1"/>
    </body>'''
        
        return {
            'body': body_xml,
            'assets': asset_xml,
            'actuators': '',
            'sensors': ''
        }

    def startDynamicSimulation(self):
        """Called when dynamic simulation starts."""
        super().startDynamicSimulation()
        # Terrain geometry is set via XML, no runtime updates needed.
        pass