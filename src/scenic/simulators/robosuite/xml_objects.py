# src/scenic/simulators/robosuite/xml_objects.py
"""XML-based object support for RoboSuite simulator."""

import os
import tempfile
import xml.etree.ElementTree as ET
from robosuite.models.objects import MujocoXMLObject
from robosuite.utils.mjcf_utils import xml_path_completion


class XMLObject:
    """Base class for XML-defined objects in Scenic."""
    
    def __init__(self, xml_path=None, xml_string=None):
        """Initialize with either XML path or string."""
        if xml_path:
            self.xml_source = xml_path
            self.is_file = True
        elif xml_string:
            self.xml_source = xml_string
            self.is_file = False
        else:
            raise ValueError("Either xml_path or xml_string must be provided")
    
    def get_xml_string(self):
        """Get XML content as string."""
        if self.is_file:
            with open(xml_path_completion(self.xml_source), 'r') as f:
                return f.read()
        else:
            return self.xml_source
    
    def modify_xml(self, modifications):
        """Apply modifications to XML before creating object."""
        # Parse XML
        tree = ET.fromstring(self.get_xml_string())
        
        # Apply modifications
        for mod_type, params in modifications.items():
            if mod_type == 'set_color':
                self._set_xml_color(tree, params['rgba'])
            elif mod_type == 'set_size':
                self._set_xml_size(tree, params['size'])
            elif mod_type == 'set_position':
                self._set_xml_position(tree, params['pos'])
        
        return ET.tostring(tree, encoding='unicode')
    
    def _set_xml_color(self, tree, rgba):
        """Set color of all geoms in XML."""
        for geom in tree.findall('.//geom'):
            if 'rgba' in geom.attrib or geom.get('group') == '1':
                geom.set('rgba', ' '.join(map(str, rgba)))
    
    def _set_xml_size(self, tree, size):
        """Set size parameters in XML."""
        for geom in tree.findall('.//geom'):
            geom_type = geom.get('type', 'box')
            if geom_type == 'box' and len(size) >= 3:
                geom.set('size', f"{size[0]/2} {size[1]/2} {size[2]/2}")
            elif geom_type == 'sphere' and len(size) >= 1:
                geom.set('size', str(size[0]/2))
            elif geom_type == 'cylinder' and len(size) >= 2:
                geom.set('size', f"{size[0]/2} {size[1]/2}")
    
    def _set_xml_position(self, tree, pos):
        """Set position of main body in XML."""
        body = tree.find('.//body')
        if body is not None:
            body.set('pos', ' '.join(map(str, pos)))


def create_xml_object(scenic_obj, name):
    """Create RoboSuite XMLObject from Scenic object with XML."""
    xml_obj = XMLObject(
        xml_path=getattr(scenic_obj, 'xml_path', None),
        xml_string=getattr(scenic_obj, 'xml_string', None)
    )
    
    # Collect modifications from Scenic properties
    modifications = {}
    
    # Color modification
    if hasattr(scenic_obj, 'color'):
        rgba = list(scenic_obj.color[:3]) + [1.0]
        modifications['set_color'] = {'rgba': rgba}
    
    # Size modification
    if hasattr(scenic_obj, 'xml_size') and scenic_obj.xml_size:
        modifications['set_size'] = {'size': scenic_obj.xml_size}
    
    # Get modified XML
    if modifications:
        xml_content = xml_obj.modify_xml(modifications)
    else:
        xml_content = xml_obj.get_xml_string()
    
    # Write to temporary file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
        f.write(xml_content)
        temp_path = f.name
    
    try:
        # Create MujocoXMLObject
        return MujocoXMLObject(temp_path, name=name)
    finally:
        # Clean up temp file
        os.unlink(temp_path)