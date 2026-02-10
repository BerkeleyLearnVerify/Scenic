import math
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET
import re

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector
from scenic.core.sensors import RGBSensor, SSSensor

from .sensors import MujocoRGBSensor, MujocoSSSensor, MujocoRenderer

VERTICAL_THRESHOLD = 0.99 # Threshold to detect gimbal lock when camera looks straight up and down.
MIN_VECTOR_LENGTH = 0.001

class MujocoSimulator(Simulator):
    """
    Class that serves as the interface between Scenic and the MuJoCo physics engine.
    Configures the simulation environment (loading the base XML and setting the viewer preferences).
    Spawns the individual `MujocoSimulation` instances for each scenario run.
    """
    
    def __init__(self, base_xml_file=None, actual=False, use_viewer=True):
        super().__init__()
        self.base_xml_file = base_xml_file
        self.actual = actual
        self.use_viewer = use_viewer
    
    def createSimulation(self, scene, **kwargs):
        return MujocoSimulation(scene, self.base_xml_file, self.actual, self.use_viewer, **kwargs)

class MujocoSimulation(Simulation):
    """
    Manages a single execution of a Scenic scenario within MuJoCo.
    Handles the lifecycle of a simulation run which includes
        - Scene Geneeration
        - XML Injection
        - Sensor Management
        - Physics Stepping
    Uses string manipulation and regex to inject object XML into the
    base model. Assumes target bodies for cameras/sensors are "leaf"
    nodes or strictly defined containers. Complex nesting in `get_mujoco_xml`
    may require careful structure to avoid regex misplacement.
    """

    def __init__(self, scene, base_xml_file=None, actual=False, use_viewer=True, **kwargs):
        self.base_xml_file = base_xml_file
        self.actual = actual
        self.use_viewer = use_viewer
        self.scene = scene

        self.mujocohandle = None
        
        # Shared renderer management to prevent memory leaks.
        # Key: (width, height) / Value: renderer
        self.shared_renderers = {}
        self.camera_counter = 0

        kwargs.pop("timestep")

        # Default timestep.
        self.timestep = 0.01
        if (scene.params.get("timestep")):
            self.timestep = scene.params.get("timestep")
        
        # Set defaults.
        if 'maxSteps' not in kwargs:
            kwargs['maxSteps'] = 1000000
        if 'name' not in kwargs:
            kwargs['name'] = 'MujocoSimulation'
        
        super().__init__(scene, timestep=self.timestep, **kwargs)

    def createObjectInSimulator(self, obj):
        # Objects are created in setup() through XML templates.
        pass

    def setup(self):
        super().setup()

        # Load base XML.
        if self.base_xml_file:
            with open(self.base_xml_file, 'r') as f:
                complete_xml = f.read()
        else:
            complete_xml = self._get_minimal_base_xml()
        
        # Process all objects with get_mujoco_xml and their sensors.
        all_bodies = []
        all_actuators = []
        all_sensors = []
        all_cameras = []
        all_assets = []
        
        object_counter = 0
        
        for obj in self.objects:
            if callable(getattr(obj, 'get_mujoco_xml')):
                
                # Get position and orientation from Scenic.
                pos_str = f"{float(obj.position.x)} {float(obj.position.y)} {float(obj.position.z)}"
                quat_str = self._get_quaternion_string(obj)
                
                # Get XML from the object's get_mujoco_xml method.
                xml_result = obj.get_mujoco_xml(object_counter, pos_str, quat_str)
                
                # Parse the XML block into sections.
                if isinstance(xml_result, str):
                    sections = self._parse_single_xml_block(xml_result)
                else:
                    sections = xml_result
                
                # Add to appropriate sections.
                if 'body' in sections and sections['body']:
                    all_bodies.append(sections['body'])
                if 'actuators' in sections and sections['actuators']:
                    all_actuators.append(sections['actuators'])
                if 'sensors' in sections and sections['sensors']:
                    all_sensors.append(sections['sensors'])
                if 'assets' in sections and sections['assets']:
                    all_assets.append(sections['assets'])
                
                # Process object sensors.
                camera_data = self._process_object_sensors(obj, object_counter)
                if camera_data:
                    all_cameras.extend(camera_data)
                
                object_counter += 1
        
        # Insert assets first.
        if all_assets:
            assets_xml = '\n'.join(all_assets)
            complete_xml = complete_xml.replace('</asset>', f'{assets_xml}\n</asset>')
        
        # Insert bodies.
        if all_bodies:
            bodies_xml = '\n'.join(all_bodies)
            complete_xml = complete_xml.replace('</worldbody>', f'{bodies_xml}\n</worldbody>')
        
        # Insert cameras into parent bodies.
        if all_cameras:
            for camera_xml, parent_body_name in all_cameras:
                
                # WARNING: This regex finds the *first* </body> tag after the opening tag.
                # It does NOT handle nested bodies correctly. Ensure 'parent_body_name' 
                # refers to a leaf node or a body containing only geoms/sites, not other bodies.
                pattern = f'(<body name="{parent_body_name}".*?)(</body>)'
                
                def insert_camera(match):
                    return match.group(1) + '\n' + camera_xml + '\n    ' + match.group(2)
                
                complete_xml = re.sub(pattern, insert_camera, complete_xml, flags=re.DOTALL)
        
        if all_actuators:
            actuators_xml = '\n'.join(all_actuators)
            complete_xml = complete_xml.replace('</actuator>', f'{actuators_xml}\n</actuator>')
        
        if all_sensors:
            sensors_xml = '\n'.join(all_sensors)
            complete_xml = complete_xml.replace('</sensor>', f'{sensors_xml}\n</sensor>')
        
        # Load into MuJoCo.
        try:
            self.model = mujoco.MjModel.from_xml_string(complete_xml)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            print(f"ERROR: MuJoCo model creation failed: {e}")
            raise

        self._initialize_sensors()
        
        # Launch viewer.
        if self.use_viewer:
            try:
                self.mujocohandle = mujoco.viewer.launch_passive(self.model, self.data)
            except Exception as e:
                print(f"WARNING: Viewer launch failed: {e}")
                self.mujocohandle = None
        else:
            self.mujocohandle = None

    def _process_object_sensors(self, obj, obj_counter):
        camera_xmls = []
        
        sensors = getattr(obj, 'sensors', None)
        if not sensors:
            return camera_xmls
            
        for _, sensor in obj.sensors.items():
            if isinstance(sensor, (RGBSensor, SSSensor)):
                camera_xml = self._generate_camera_xml(sensor, obj_counter)
                if camera_xml:
                    camera_xmls.append((camera_xml, obj.body_name))
                        
        return camera_xmls

    def _generate_camera_xml(self, sensor, obj_counter):

        attrs = getattr(sensor, 'attributes', None) or {}

        # Get offset.
        offset = getattr(sensor, 'offset', None) or (3.0, 2.0, 2.0)
        offset_x, offset_y, offset_z = offset

        # Get field of view.
        fovy = attrs.get('fovy', 60)
        
        # Set look direction from attributes; default to looking forward.
        look_x, look_y, look_z = attrs.get('look_direction', (1.0, 0.0, 0.0))

        # Create camera name.
        self.camera_counter += 1
        camera_name = f"camera_{obj_counter}_{self.camera_counter}"
        sensor.camera_name = camera_name
        
        # Normalize the look direction.
        length = math.sqrt(look_x*look_x + look_y*look_y + look_z*look_z)
        if length > 0:
            look_x /= length
            look_y /= length
            look_z /= length
        
        # Calculate camera axes for MuJoCo xyaxes.
        # Check if looking nearly straight up or down.
        if abs(look_z) > VERTICAL_THRESHOLD:  # Nearly vertical.
            if look_z < 0:  # Looking down.
                xyaxes = "1 0 0 0 1 0"
            else:  # Looking up.
                xyaxes = "1 0 0 0 -1 0"
        else:
            # Normal case: calculate right and up vectors.
            world_up_x, world_up_y, world_up_z = 0, 0, 1
            
            right_x = look_y * world_up_z - look_z * world_up_y
            right_y = look_z * world_up_x - look_x * world_up_z
            right_z = look_x * world_up_y - look_y * world_up_x
            
            right_length = math.sqrt(right_x*right_x + right_y*right_y + right_z*right_z)
            if right_length > MIN_VECTOR_LENGTH:
                right_x /= right_length
                right_y /= right_length
                right_z /= right_length
            else:
                # Fallback if right vector is too small.
                right_x, right_y, right_z = 0, 1, 0
            
            up_x = right_y * look_z - right_z * look_y
            up_y = right_z * look_x - right_x * look_z
            up_z = right_x * look_y - right_y * look_x
            
            up_length = math.sqrt(up_x*up_x + up_y*up_y + up_z*up_z)
            if up_length > MIN_VECTOR_LENGTH:
                up_x /= up_length
                up_y /= up_length
                up_z /= up_length
            
            xyaxes = f"{right_x:.3f} {right_y:.3f} {right_z:.3f} {up_x:.3f} {up_y:.3f} {up_z:.3f}"
        
        # Return camera XML with relative position.
        camera_xml = f'''        <camera name="{camera_name}" pos="{offset_x:.3f} {offset_y:.3f} {offset_z:.3f}" xyaxes="{xyaxes}" fovy="{fovy}"/>'''
        
        return camera_xml

    def _build_geom_to_object_mapping(self):
        """Build mapping from geom_id to Scenic object for semantic segmentation."""
        geom_to_object = {}
        
        for obj in self.objects:
            # Get the instance_id to match geom names.
            if getattr(obj, 'instance_id', None):
                instance_id = obj.instance_id
                
                # Iterate through all geoms in the model.
                for geom_id in range(self.model.ngeom):
                    geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
                    
                    # Check if this geom belongs to this object by checking if instance_id is in the name.
                    if geom_name and instance_id in geom_name:
                        geom_to_object[geom_id] = obj
        
        return geom_to_object

    def _initialize_sensors(self):
        """Initialize all sensors with shared renderers and geom-to-object mappings."""
        
        # Build geom-to-object mapping for semantic segmentation.
        geom_to_object_map = self._build_geom_to_object_mapping()
        
        for obj in self.objects:
            if getattr(obj, 'sensors', None):
                for sensor_name, sensor in obj.sensors.items():
                    if isinstance(sensor, (MujocoRGBSensor, MujocoSSSensor)):
                        sensor.model = self.model
                        sensor.data = self.data
                        
                        # For SS sensors, provide geom-to-object mapping.
                        if isinstance(sensor, MujocoSSSensor):
                            sensor.geom_to_object = geom_to_object_map
                        
                        # Use shared renderer based on resolution.
                        key = (sensor.width, sensor.height)
                        if key not in self.shared_renderers:
                            renderer = MujocoRenderer(self.model, sensor.width, sensor.height)
                            if renderer.is_initialized():
                                self.shared_renderers[key] = renderer
                            else:
                                print(f"ERROR: Failed to create renderer for {key}")
                                continue
                        
                        sensor.renderer = self.shared_renderers[key]
                        
                        # Verify camera exists in model.
                        if sensor.camera_name:
                            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, sensor.camera_name)
                            if camera_id == -1:
                                print(f"WARNING: Camera '{sensor.camera_name}' not found in model")
    
    def _parse_single_xml_block(self, xml_block):
        """Parse a single XML block into body, actuators, and sensors sections."""
        sections = {'body': '', 'actuators': '', 'sensors': ''}
        
        try:
            # Wrap in a root element for parsing.
            root_xml = f"<root>{xml_block}</root>"
            root = ET.fromstring(root_xml)
            
            # Extract all top-level body elements.
            body_elements = []
            for child in root:
                if child.tag == 'body':
                    body_elements.append(ET.tostring(child, encoding='unicode'))
            
            if body_elements:
                sections['body'] = '\n'.join(body_elements)
            
            # Extract actuators section.
            actuators_elements = root.findall('.//actuators')
            if actuators_elements:
                actuators_content = []
                for actuator_section in actuators_elements:
                    for child in actuator_section:
                        actuators_content.append(ET.tostring(child, encoding='unicode'))
                if actuators_content:
                    sections['actuators'] = '\n'.join(actuators_content)
            
            # Extract sensors section.
            sensors_elements = root.findall('.//sensors')
            if sensors_elements:
                sensors_content = []
                for sensor_section in sensors_elements:
                    for child in sensor_section:
                        sensors_content.append(ET.tostring(child, encoding='unicode'))
                if sensors_content:
                    sections['sensors'] = '\n'.join(sensors_content)
                    
        except ET.ParseError as e:
            print(f"Warning: Failed to parse XML block: {e}")
            # Fallback: treat entire block as body.
            sections['body'] = xml_block
        
        return sections

    def _get_quaternion_string(self, obj):
        q = obj.orientation.q
        return f"{q[0]} {q[1]} {q[2]} {q[3]}"

    def _get_minimal_base_xml(self):
        return '''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="scenic_model">
    <option timestep="0.002" integrator="RK4"/>
    
    <visual>
        <global offwidth="1024" offheight="768"/>
    </visual>
    
    <default>
        <geom condim="4" friction="1.0 0.1 0.01"/>
        <joint armature="0.0005" damping="0.1"/>
    </default>
    
    <asset>
        <texture type="skybox" builtin="gradient" rgb1=".4 .6 .8" rgb2=".0 .0 .2" width="800" height="800"/>
        <texture name="groundplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 .2 .3" width="300" height="300"/>
        <material name="MatPlane" reflectance="0.5" texture="groundplane" texrepeat="60 60" texuniform="true"/>
    </asset>
    
    <worldbody>
        <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" pos="0 0 1.3" specular=".1 .1 .1"/>
        <geom name="ground" type="plane" conaffinity="1" condim="3" material="MatPlane" pos="0 0 0" size="40 40 40"/>
    </worldbody>
    
    <actuator>
    </actuator>
    
    <sensor>
    </sensor>
</mujoco>'''
    
    def step(self):
        if self.mujocohandle and not self.mujocohandle.is_running():
            raise KeyboardInterrupt("MuJoCo Viewer closed by user.")

        # Let objects control themselves.
        for obj in self.objects:
            if getattr(obj, 'control', None):
                obj.control(self.model, self.data)
        
        mujoco.mj_step(self.model, self.data)
        
        if self.mujocohandle:
            self.mujocohandle.sync()

        self._update_observations()

    def _update_observations(self):
        """Update observations with proper sensor coordination to prevent double rendering."""
        
        for obj in self.objects:
            if not getattr(obj, 'sensors', None):
                continue
                
            if not getattr(obj, 'observations', None):
                obj.observations = {}
            
            # Categorize sensors by type for ordered processing.
            # RGB sensors must be processed first to establish shared cache for SS sensors.
            sensor_groups = {
                'rgb': [],
                'ss': [],
                'other': []
            }
            
            for sensor_name, sensor in obj.sensors.items():
                if isinstance(sensor, MujocoRGBSensor):
                    sensor_groups['rgb'].append((sensor_name, sensor))
                elif isinstance(sensor, MujocoSSSensor):
                    sensor_groups['ss'].append((sensor_name, sensor))
                else:
                    sensor_groups['other'].append((sensor_name, sensor))
            
            # Process in order: RGB -> SS -> other
            # This ensures SS sensors can reuse RGB rendering cache.
            for group_name in ['rgb', 'ss', 'other']:
                for sensor_name, sensor in sensor_groups[group_name]:
                    try:
                        obj.observations[sensor_name] = sensor.getObservation()
                    except Exception as e:
                        print(f"Failed to update {group_name.upper()} sensor {sensor_name}: {e}")
                        obj.observations[sensor_name] = None

    def getProperties(self, obj, properties):
        if not getattr(obj, 'body_name', None):
        # Return minimal properties for non-body objects (like Terrain).
            return {
                'position': obj.position,
                'velocity': Vector(0, 0, 0),
                'speed': 0,
                'angularSpeed': 0,
                'angularVelocity': Vector(0, 0, 0),
                'yaw': 0,
                'pitch': 0,
                'roll': 0,
            }
    
        try:
            body_data = self.data.body(obj.body_name)
            
            # Position
            x, y, z = body_data.subtree_com
            position = Vector(x, y, z)
            
            # Velocity
            vel_x, vel_y, vel_z = body_data.cvel[0:3]
            velocity = Vector(vel_x, vel_y, vel_z)
            speed = math.hypot(*velocity)
            
            # Angular velocity
            ang_x, ang_y, ang_z = body_data.cvel[3:6]
            angularVelocity = Vector(ang_x, ang_y, ang_z)
            angularSpeed = math.hypot(*angularVelocity)
            
            # Orientation
            try:
                orientation_matrix = body_data.ximat.reshape(3, 3)
                r = Rotation.from_matrix(orientation_matrix)
                yaw, pitch, roll = r.as_euler('zyx')
            except:
                yaw = pitch = roll = 0.0
            
            return {
                'position': position,
                'velocity': velocity,
                'speed': speed,
                'angularSpeed': angularSpeed,
                'angularVelocity': angularVelocity,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll,
            }
            
        except Exception as e:
            print(f"Error getting properties for {obj}: {e}")
            return {}

    def destroy(self):
        """Clean up all resources properly."""
        
        # Clean up shared renderers first.
        for key, renderer in self.shared_renderers.items():
            try:
                renderer.close()
            except Exception as e:
                print(f"WARNING: Error closing renderer {key}: {e}")
        
        self.shared_renderers.clear()
        
        # Close MuJoCo viewer.
        if self.mujocohandle:
            try:
                self.mujocohandle.close()
            except Exception as e:
                print(f"WARNING: Error closing viewer: {e}")
            self.mujocohandle = None
        
        # Call parent cleanup.
        super().destroy()