# src/scenic/simulators/robosuite/xml_builder.py
"""XML builder for RoboSuite-Scenic integration."""

import os
import tempfile
import xml.etree.ElementTree as ET
from robosuite.models.objects import MujocoObject
from robosuite.models.arenas import Arena
from robosuite.utils.mjcf_utils import array_to_string, xml_path_completion


class XMLModifiableObject(MujocoObject):
    """MujocoObject that can be modified at runtime."""
    
    def __init__(self, name, xml_path=None, xml_string=None, **kwargs):
        self.name = self.naming_prefix = name
        self.obj_type = kwargs.get('obj_type', 'all')
        
        if xml_string:
            self.root = ET.fromstring(xml_string)
            self.worldbody = self.root.find("worldbody")
            self.asset = self.root.find("asset")
        else:
            super().__init__(xml_path_completion(xml_path))
    
    def get_obj(self):
        """Get the XML tree with name prefix."""
        from robosuite.utils.mjcf_utils import add_prefix
        add_prefix(self.root, prefix=self.naming_prefix)
        return self.root


class RoboSuiteXMLBuilder:
    """Builder for creating RoboSuite objects from Scenic specifications."""
    
    def create_scenic_object(self, scenic_obj):
        """Create RoboSuite object from Scenic object."""
        name = f"{type(scenic_obj).__name__}_{id(scenic_obj)}"
        
        # XML-based object
        if hasattr(scenic_obj, 'xml_path') and scenic_obj.xml_path:
            obj = XMLModifiableObject(name, xml_path=scenic_obj.xml_path)
        elif hasattr(scenic_obj, 'xml_string') and scenic_obj.xml_string:
            obj = XMLModifiableObject(name, xml_string=scenic_obj.xml_string)
        else:
            # Procedural object
            obj = XMLModifiableObject(name, xml_string=self._build_procedural_xml(scenic_obj, name))
        
        # Apply properties
        self._apply_properties(obj, scenic_obj)
        return obj
    
    def create_arena_from_xml(self, xml_source, is_string=True):
        """Create arena from XML string or path."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            if is_string:
                f.write(xml_source)
            else:
                with open(xml_path_completion(xml_source)) as src:
                    f.write(src.read())
            temp_path = f.name
        
        try:
            return Arena(temp_path)
        finally:
            os.unlink(temp_path)
    
    def _build_procedural_xml(self, scenic_obj, name):
        """Build XML for geometric objects."""
        obj_type = type(scenic_obj).__name__.lower()
        
        # Geometry
        if 'ball' in obj_type or 'sphere' in obj_type:
            geom_type, size_str = "sphere", str(scenic_obj.width/2)
        elif 'cylinder' in obj_type:
            geom_type, size_str = "cylinder", f"{scenic_obj.width/2} {scenic_obj.height/2}"
        else:  # box/cube
            geom_type, size_str = "box", f"{scenic_obj.width/2} {scenic_obj.length/2} {scenic_obj.height/2}"
        
        # Properties
        rgba = array_to_string(list(getattr(scenic_obj, 'color', [0.5, 0.5, 0.5])) + [1.0])
        density = getattr(scenic_obj, 'density', 1000)
        friction = array_to_string(getattr(scenic_obj, 'friction', [1.0, 0.005, 0.0001]))
        
        return f"""<?xml version="1.0"?>
        <mujoco model="{name}">
            <worldbody>
                <body name="{name}_main">
                    <geom name="{name}_collision" type="{geom_type}" size="{size_str}" 
                          pos="0 0 0" group="0" rgba="{rgba}"
                          density="{density}" friction="{friction}"/>
                    <geom name="{name}_visual" type="{geom_type}" size="{size_str}" 
                          pos="0 0 0" group="1" rgba="{rgba}"/>
                </body>
            </worldbody>
        </mujoco>"""
    
    def _apply_properties(self, obj, scenic_obj):
        """Apply Scenic properties to object."""
        # Color
        if hasattr(scenic_obj, 'color'):
            rgba = array_to_string(list(scenic_obj.color[:3]) + [1.0])
            for geom in obj.worldbody.findall(".//geom[@group='1']") or []:
                geom.set("rgba", rgba)
                if "material" in geom.attrib:
                    del geom.attrib["material"]
        
        # Physics
        if hasattr(scenic_obj, 'density'):
            for geom in obj.worldbody.findall(".//geom[@group='0']") or []:
                geom.set("density", str(scenic_obj.density))
        
        if hasattr(scenic_obj, 'friction'):
            for geom in obj.worldbody.findall(".//geom[@group='0']") or []:
                geom.set("friction", array_to_string(scenic_obj.friction))