# src/scenic/simulators/robosuite/xml_builder.py
"""XML manipulation using RoboSuite's built-in tools."""

import numpy as np
import tempfile
import os
import xml.etree.ElementTree as ET
from robosuite.models import MujocoWorldBase
from robosuite.models.arenas import Arena
from robosuite.models.objects import MujocoXMLObject, MujocoObject
from robosuite.utils import mjcf_utils
from robosuite.utils.mjcf_utils import (
    array_to_string, string_to_array, find_elements, 
    add_prefix, new_body, new_geom, add_material,
    CustomMaterial, xml_path_completion
)
from scenic.core.vectors import Vector


class XMLModifiableObject(MujocoObject):
    """RoboSuite object that can be modified via Scenic properties."""
    
    def __init__(self, name, xml_path=None, xml_string=None, joints=None,
                 obj_type="all", duplicate_collision_geoms=True):
        """
        Initialize with either XML path or string.
        
        Args:
            xml_path: Path to XML file
            xml_string: XML content as string
            joints: Joint specifications
            obj_type: Object type for RoboSuite
            duplicate_collision_geoms: Whether to duplicate collision geoms
        """
        # Store XML source
        self._xml_source = xml_string if xml_string else xml_path
        self._is_string = xml_string is not None
        
        # Parse XML to get object tree
        if self._is_string:
            # Write XML string to temp file for MujocoObject
            with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
                f.write(xml_string)
                temp_fname = f.name
            fname = temp_fname
        else:
            fname = xml_path_completion(xml_path)
            temp_fname = None
            
        # Initialize parent class with file
        super().__init__(
            fname,
            name,
            joints,
            obj_type,
            duplicate_collision_geoms
        )
        
        # Clean up temp file if created
        if temp_fname:
            os.unlink(temp_fname)
        
    def set_color(self, rgba):
        """Set color of all visual geoms."""
        # Find all visual geoms (group 1)
        visual_geoms = find_elements(
            self.worldbody, 
            tags="geom",
            attribs={"group": "1"},
            return_first=False
        )
        
        # Update rgba for each visual geom
        for geom in visual_geoms or []:
            geom.set("rgba", array_to_string(rgba))
            # Remove material if exists (color overrides material)
            if "material" in geom.attrib:
                del geom.attrib["material"]
    
    def set_size(self, size_params):
        """Update size of geoms based on object type."""
        # Get all geoms
        geoms = find_elements(self.worldbody, tags="geom", return_first=False)
        
        for geom in geoms or []:
            geom_type = geom.get("type", "box")
            
            if geom_type == "box" and len(size_params) >= 3:
                # Box takes half-sizes
                geom.set("size", f"{size_params[0]/2} {size_params[1]/2} {size_params[2]/2}")
            elif geom_type == "sphere" and len(size_params) >= 1:
                # Sphere takes radius
                geom.set("size", str(size_params[0]/2))
            elif geom_type == "cylinder" and len(size_params) >= 2:
                # Cylinder takes radius and half-height
                geom.set("size", f"{size_params[0]/2} {size_params[1]/2}")
    
    def add_custom_material(self, material_name, texture_path=None, **mat_attribs):
        """Add custom material to the object."""
        if not self.asset:
            self.asset = ET.SubElement(self.root, "asset")
        
        # Create custom material
        if texture_path:
            custom_mat = CustomMaterial(
                texture=texture_path,
                tex_name=f"{material_name}_tex",
                mat_name=material_name,
                mat_attrib=mat_attribs
            )
        else:
            # Material without texture
            custom_mat = CustomMaterial(
                texture=None,
                tex_name=None,
                mat_name=material_name,
                mat_attrib=mat_attribs
            )
        
        # Add material to XML
        tex_element, mat_element, _, _ = add_material(
            self.root,
            naming_prefix=self.naming_prefix,
            custom_material=custom_mat
        )
    
    def get_obj(self):
        """Override to return modified tree."""
        # Add name prefix to all elements
        add_prefix(self.root, prefix=self.naming_prefix)
        
        # Return the modified root element
        return self.root


class RoboSuiteXMLBuilder:
    """Build and manipulate MuJoCo models using RoboSuite's tools."""
    
    def __init__(self):
        self.custom_arena = None
        self.xml_objects = []
        
    def create_positionable_table(self, position, size, orientation=None):
        """Create table arena with position control."""
        # Table positioning: position.z is where the table legs touch the ground
        table_thickness = 0.05
        
        # Get RoboSuite's asset path
        import robosuite
        robosuite_path = os.path.dirname(robosuite.__file__)
        texture_path = os.path.join(robosuite_path, "models/assets/textures")
        
        # Create custom XML with proper textures and materials
        table_xml = f"""<?xml version="1.0" encoding="utf-8"?>
        <mujoco>
            <compiler angle="radian" coordinate="local" meshdir="meshes/"/>
            <option timestep="0.002" integrator="RK4" gravity="0 0 -9.81"/>
            
            <visual>
                <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8" specular="0.1 0.1 0.1"/>
                <map znear="0.01" shadowclip="0.5"/>
                <quality shadowsize="2048"/>
                <global offwidth="800" offheight="600"/>
            </visual>
            
            <asset>
                <!-- Sky texture -->
                <texture name="skybox" type="skybox" builtin="gradient" rgb1="0.9 0.9 1.0" rgb2="0.2 0.3 0.4" width="256" height="256"/>
                
                <!-- Floor texture - using RoboSuite's texture file -->
                <texture file="{texture_path}/light-gray-floor-tile.png" type="2d" name="texplane"/>
                <material name="floorplane" texture="texplane" texrepeat="2 2" specular="0.0" shininess="0.0" reflectance="0.01" texuniform="true"/>
                
                <!-- Table material - ceramic texture -->
                <texture file="{texture_path}/ceramic.png" type="cube" name="tex-ceramic"/>
                <material name="table_mat" texture="tex-ceramic" specular="0.2" shininess="0.0" reflectance="0.0" texrepeat="1 1"/>
                
                <!-- Leg material - steel texture -->
                <texture file="{texture_path}/steel-brushed.png" type="cube" name="tex-steel-brushed"/>
                <material name="leg_mat" texture="tex-steel-brushed" specular="0.8" shininess="0.8" reflectance="0.8" texrepeat="1 1"/>
            </asset>
            
            <worldbody>
                <!-- Table positioned so bottom of legs is at position.z -->
                <body name="table" pos="{position.x} {position.y} {position.z}">
                    <!-- Table top (white) -->
                    <geom name="table_collision" type="box" 
                          pos="0 0 {size[2] - table_thickness/2}"
                          size="{size[0]/2} {size[1]/2} {table_thickness/2}" 
                          friction="1 0.005 0.0001"/>
                    <geom name="table_visual" type="box" 
                          pos="0 0 {size[2] - table_thickness/2}"
                          size="{size[0]/2} {size[1]/2} {table_thickness/2}" 
                          material="table_mat" conaffinity="0" contype="0" group="1"/>
                    
                    <!-- Table top site -->
                    <site name="table_top" pos="0 0 {size[2]}" size="0.001 0.001 0.001" rgba="0 0 0 0"/>
                    
                    <!-- Legs (grey) -->
                    <geom name="table_leg1_visual" type="cylinder" 
                          pos="{size[0]/2-0.1} {size[1]/2-0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          material="leg_mat" conaffinity="0" contype="0" group="1"/>
                    <geom name="table_leg2_visual" type="cylinder" 
                          pos="{-size[0]/2+0.1} {size[1]/2-0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          material="leg_mat" conaffinity="0" contype="0" group="1"/>
                    <geom name="table_leg3_visual" type="cylinder" 
                          pos="{size[0]/2-0.1} {-size[1]/2+0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          material="leg_mat" conaffinity="0" contype="0" group="1"/>
                    <geom name="table_leg4_visual" type="cylinder" 
                          pos="{-size[0]/2+0.1} {-size[1]/2+0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          material="leg_mat" conaffinity="0" contype="0" group="1"/>
                </body>
                
                <!-- Floor plane -->
                <geom name="floor" type="plane" pos="0 0 0" size="3 3 0.125" 
                      material="floorplane" condim="3" group="1"/>
                
                <!-- Lighting -->
                <light name="light1" pos="1.0 1.0 1.5" dir="-0.2 -0.2 -1" 
                       directional="true" specular="0.3 0.3 0.3" castshadow="false"/>
            </worldbody>
        </mujoco>
        """
        
        # Write XML to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(table_xml)
            temp_path = f.name
        
        try:
            # Create Arena from the temporary file
            arena = Arena(temp_path)
            return arena
        finally:
            # Clean up temp file
            os.unlink(temp_path)
    
    def create_multi_table_arena(self, tables):
        """Create arena with multiple tables."""
        print(f"DEBUG: Creating arena with {len(tables)} tables")
        for i, (pos, size) in enumerate(tables):
            print(f"  Table {i}: pos={pos}, size={size}")
            
        import robosuite
        robosuite_path = os.path.dirname(robosuite.__file__)
        texture_path = os.path.join(robosuite_path, "models/assets/textures")
        
        # Build table bodies
        table_bodies = ""
        for i, (position, size) in enumerate(tables):
            table_thickness = 0.05
            table_bodies += f"""
                <!-- Table {i} -->
                <body name="table_{i}" pos="{position.x} {position.y} {position.z}">
                    <!-- Table top -->
                    <geom name="table_{i}_collision" type="box" 
                          pos="0 0 {size[2] - table_thickness/2}"
                          size="{size[0]/2} {size[1]/2} {table_thickness/2}" 
                          friction="1 0.005 0.0001" rgba="0.9 0.9 0.9 1"/>
                    <geom name="table_{i}_visual" type="box" 
                          pos="0 0 {size[2] - table_thickness/2}"
                          size="{size[0]/2} {size[1]/2} {table_thickness/2}" 
                          rgba="0.95 0.95 0.95 1" conaffinity="0" contype="0" group="1"/>
                    
                    <!-- Legs -->
                    <geom name="table_{i}_leg1" type="cylinder" 
                          pos="{size[0]/2-0.1} {size[1]/2-0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" rgba="0.5 0.5 0.5 1"/>
                    <geom name="table_{i}_leg2" type="cylinder" 
                          pos="{-size[0]/2+0.1} {size[1]/2-0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" rgba="0.5 0.5 0.5 1"/>
                    <geom name="table_{i}_leg3" type="cylinder" 
                          pos="{size[0]/2-0.1} {-size[1]/2+0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" rgba="0.5 0.5 0.5 1"/>
                    <geom name="table_{i}_leg4" type="cylinder" 
                          pos="{-size[0]/2+0.1} {-size[1]/2+0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" rgba="0.5 0.5 0.5 1"/>
                </body>
            """
        
        arena_xml = f"""<?xml version="1.0" encoding="utf-8"?>
        <mujoco>
            <visual>
                <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8" specular="0.1 0.1 0.1"/>
            </visual>
            
            <asset>
                <texture file="{texture_path}/light-gray-floor-tile.png" type="2d" name="texplane"/>
                <material name="floorplane" texture="texplane" texrepeat="2 2" specular="0.0" shininess="0.0"/>
            </asset>
            
            <worldbody>
                {table_bodies}
                
                <!-- Floor -->
                <geom name="floor" type="plane" pos="0 0 0" size="3 3 0.125" material="floorplane" condim="3" group="1"/>
            </worldbody>
        </mujoco>
        """
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(arena_xml)
            temp_path = f.name
        
        try:
            arena = Arena(temp_path)
            return arena
        finally:
            os.unlink(temp_path)
    
    def create_positionable_table_object(self, size, name="table", orientation=None):
        """Create table as MujocoObject for multiple tables."""
        table_thickness = 0.05
        
        # Create table XML as object - position will be set later with set_base_xpos
        table_xml = f"""<?xml version="1.0" encoding="utf-8"?>
        <mujoco model="{name}">
            <worldbody>
                <body name="{name}_main">
                    <!-- Table top -->
                    <geom name="{name}_collision" type="box" 
                          pos="0 0 {size[2] - table_thickness/2}"
                          size="{size[0]/2} {size[1]/2} {table_thickness/2}" 
                          friction="1 0.005 0.0001" rgba="0.9 0.9 0.9 1"/>
                    <geom name="{name}_visual" type="box" 
                          pos="0 0 {size[2] - table_thickness/2}"
                          size="{size[0]/2} {size[1]/2} {table_thickness/2}" 
                          rgba="0.95 0.95 0.95 1" conaffinity="0" contype="0" group="1"/>
                    
                    <!-- Table top site -->
                    <site name="{name}_top" pos="0 0 {size[2]}" size="0.001 0.001 0.001" rgba="0 0 0 0"/>
                    
                    <!-- Legs -->
                    <geom name="{name}_leg1" type="cylinder" 
                          pos="{size[0]/2-0.1} {size[1]/2-0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          rgba="0.5 0.5 0.5 1"/>
                    <geom name="{name}_leg2" type="cylinder" 
                          pos="{-size[0]/2+0.1} {size[1]/2-0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          rgba="0.5 0.5 0.5 1"/>
                    <geom name="{name}_leg3" type="cylinder" 
                          pos="{size[0]/2-0.1} {-size[1]/2+0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          rgba="0.5 0.5 0.5 1"/>
                    <geom name="{name}_leg4" type="cylinder" 
                          pos="{-size[0]/2+0.1} {-size[1]/2+0.1} {(size[2]-table_thickness)/2}"
                          size="0.025 {(size[2]-table_thickness)/2}" 
                          rgba="0.5 0.5 0.5 1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        
        # Write to temp file and create MujocoXMLObject
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(table_xml)
            temp_path = f.name
        
        try:
            return MujocoXMLObject(temp_path, name=name)
        finally:
            os.unlink(temp_path)
    
    def create_scenic_object(self, scenic_obj):
        """Create RoboSuite object from Scenic object with XML support."""
        
        # Check if object has custom XML
        if hasattr(scenic_obj, 'xml_path') and scenic_obj.xml_path:
            return self._create_from_xml_file(scenic_obj)
        elif hasattr(scenic_obj, 'xml_string') and scenic_obj.xml_string:
            return self._create_from_xml_string(scenic_obj)
        else:
            # Generate XML procedurally
            return self._create_procedural_object(scenic_obj)
    
    def _create_from_xml_file(self, scenic_obj):
        """Create object from XML file with modifications."""
        name = f"{type(scenic_obj).__name__}_{id(scenic_obj)}"
        
        # Create modifiable object
        obj = XMLModifiableObject(
            name=name,
            xml_path=scenic_obj.xml_path,
            obj_type="all",
            duplicate_collision_geoms=True
        )
        
        # Apply Scenic properties
        self._apply_scenic_properties(obj, scenic_obj)
        
        return obj
    
    def _create_from_xml_string(self, scenic_obj):
        """Create object from XML string with modifications."""
        name = f"{type(scenic_obj).__name__}_{id(scenic_obj)}"
        
        # Create modifiable object
        obj = XMLModifiableObject(
            name=name,
            xml_string=scenic_obj.xml_string,
            obj_type="all",
            duplicate_collision_geoms=True
        )
        
        # Apply Scenic properties
        self._apply_scenic_properties(obj, scenic_obj)
        
        return obj
    
    def _create_procedural_object(self, scenic_obj):
        """Generate object XML procedurally using mjcf_utils."""
        obj_type = type(scenic_obj).__name__.lower()
        name = f"{obj_type}_{id(scenic_obj)}"
        
        # Create root
        root = ET.Element("mujoco")
        root.set("model", name)
        
        # Create main body
        body = new_body(name=f"{name}_main", pos=[0, 0, 0])
        
        # Determine geom type and size
        if 'cube' in obj_type or 'box' in obj_type:
            geom_type = "box"
            size = [scenic_obj.width/2, scenic_obj.length/2, scenic_obj.height/2]
        elif 'ball' in obj_type or 'sphere' in obj_type:
            geom_type = "sphere"
            size = [scenic_obj.width/2]
        elif 'cylinder' in obj_type:
            geom_type = "cylinder"
            size = [scenic_obj.width/2, scenic_obj.height/2]
        else:
            geom_type = "box"
            size = [scenic_obj.width/2, scenic_obj.length/2, scenic_obj.height/2]
        
        # Get color
        rgba = list(getattr(scenic_obj, 'color', [0.5, 0.5, 0.5])) + [1.0]
        
        # Create collision geom
        collision_geom = new_geom(
            name=f"{name}_collision",
            type=geom_type,
            size=size,
            pos=[0, 0, 0],
            group=0,
            rgba=rgba
        )
        
        # Create visual geom
        visual_geom = new_geom(
            name=f"{name}_visual",
            type=geom_type,
            size=size,
            pos=[0, 0, 0],
            group=1,
            rgba=rgba
        )
        
        # Add physics properties
        if hasattr(scenic_obj, 'density'):
            collision_geom.set("density", str(scenic_obj.density))
        if hasattr(scenic_obj, 'friction'):
            collision_geom.set("friction", array_to_string(scenic_obj.friction))
        
        # Assemble XML
        body.append(collision_geom)
        body.append(visual_geom)
        root.append(body)
        
        # Create XML string
        xml_string = ET.tostring(root, encoding='unicode')
        
        # Create object
        obj = XMLModifiableObject(
            name=name,
            xml_string=xml_string,
            obj_type="all",
            duplicate_collision_geoms=False
        )
        
        return obj
    
    def _apply_scenic_properties(self, obj, scenic_obj):
        """Apply Scenic properties to RoboSuite object."""
        
        # Set color if specified
        if hasattr(scenic_obj, 'color'):
            rgba = list(scenic_obj.color[:3]) + [1.0]
            obj.set_color(rgba)
        
        # Set size if object supports it
        if hasattr(scenic_obj, 'width'):
            size_params = []
            if hasattr(scenic_obj, 'width'):
                size_params.append(scenic_obj.width)
            if hasattr(scenic_obj, 'length'):
                size_params.append(scenic_obj.length)
            if hasattr(scenic_obj, 'height'):
                size_params.append(scenic_obj.height)
            
            if size_params:
                obj.set_size(size_params)
        
        # Add custom material if specified
        if hasattr(scenic_obj, 'material'):
            obj.add_custom_material(
                material_name=scenic_obj.material.get('name', 'custom'),
                texture_path=scenic_obj.material.get('texture'),
                **scenic_obj.material.get('attributes', {})
            )
        
        # Set physics properties
        if hasattr(scenic_obj, 'density'):
            collision_geoms = find_elements(
                obj.worldbody,
                tags="geom",
                attribs={"group": "0"},
                return_first=False
            )
            for geom in collision_geoms or []:
                geom.set("density", str(scenic_obj.density))
        
        if hasattr(scenic_obj, 'friction'):
            collision_geoms = find_elements(
                obj.worldbody,
                tags="geom", 
                attribs={"group": "0"},
                return_first=False
            )
            for geom in collision_geoms or []:
                geom.set("friction", array_to_string(scenic_obj.friction))
    
    def create_arena_from_xml_string(self, xml_string):
        """Create arena from XML string."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(xml_string)
            temp_path = f.name
        
        try:
            return Arena(temp_path)
        finally:
            os.unlink(temp_path)
    
    def create_arena_from_xml_path(self, xml_path):
        """Create arena from XML file path."""
        full_path = xml_path_completion(xml_path)
        return Arena(full_path)