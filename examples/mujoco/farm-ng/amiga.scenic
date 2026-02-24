from scenic.simulators.mujoco.model import MujocoBody, DynamicMujocoBody

class Amiga(DynamicMujocoBody):
    
    # Scenic property - can be overridden in .scenic files.
    semanticColor: [255, 0, 0]  # Default green.
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.instance_id = None
        # For torque control.
        self.target_velocity = 0.0  # Target wheel velocity (rad/s).
        self.kp = 50.0  # Proportional gain for velocity control.
        self.kd = 5.0   # Derivative gain for damping.
    
    def semanticColorForGeom(self, geom_name):
        """
        Return semantic color for a specific geom.
        
        This allows custom coloring of different parts of the Amiga.
        Override this method or use the default semanticColor for all geoms.
        
        Args:
            geom_name: Name of the geometry (e.g., "fl_wheel_amiga_0", "rail_left_amiga_0")
        
        Returns:
            [R, G, B] color list, or None to use default semanticColor
        """
        # Example: Color wheels differently from the frame.
        if 'wheel' in geom_name.lower():
            return [50, 50, 50]  # Dark gray for wheels.
        
        # # Example: Color rails and crossbars with frame color.
        # if any(part in geom_name.lower() for part in ['rail', 'crossbar', 'control_head']):
        #     return [200, 200, 200]  # Light gray for structure
        
        # # Example: Color module sides.
        # if 'module_side' in geom_name.lower():
        #     return [220, 220, 210]  # Off-white for module sides
        
        # Default: use the object's semanticColor.
        return None

    def get_mujoco_xml(self, obj_counter, position, quaternion):
        u_id = f"amiga_{obj_counter}"
        
        # Dedicated chassis body for the camera.
        self.body_name = f"chassis_{u_id}"
        self.instance_id = u_id
        
        complete_xml = f'''
    <body name="root_{u_id}" pos="{position}" quat="{quaternion}">
        <freejoint name="root_joint_{u_id}"/>

        <body name="{self.body_name}" pos="0 0 0">
            <geom name="rail_left_{u_id}" type="box" size="0.60 0.04 0.04" pos="0  0.32 0.50" rgba="0.2 0.2 0.25 1" mass="2"/>
            <geom name="rail_right_{u_id}" type="box" size="0.60 0.04 0.04" pos="0 -0.32 0.50" rgba="0.2 0.2 0.25 1" mass="2"/>

            <geom name="crossbar_front_{u_id}" type="box" size="0.04 0.52 0.04" pos=" 0.60 0 1.34" rgba="0.2 0.2 0.25 1" mass="1"/>
            <geom name="crossbar_rear_{u_id}" type="box" size="0.04 0.52 0.04" pos="-0.60 0 1.34" rgba="0.2 0.2 0.25 1" mass="1"/>

            <geom name="control_head_{u_id}" type="box" size="0.10 0.07 0.05" pos="0.45 0.35 0.95" rgba="0.1 0.1 0.1 1" mass="0.5"/>

            <site name="gps_site_{u_id}" pos="0.00 0.00 0.98" size="0.01" rgba="0 1 0 0"/>
            <site name="imu_site_{u_id}" pos="0.05 0.00 0.95" size="0.01" rgba="1 0 0 0"/>
            <site name="front_mount_{u_id}" pos="0.70 0.00 0.74" size="0.01" rgba="0 0.6 1 0"/>
            <site name="rear_mount_{u_id}" pos="-0.70 0.00 0.74" size="0.01" rgba="0 0.6 1 0"/>
        </body>

        <body name="fl_module_{u_id}" pos=" 0.60  0.50 0.40">
            <geom name="fl_module_side1_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00  0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <geom name="fl_module_side2_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00 -0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <body name="fl_wheel_body_{u_id}" pos="0 0 0">
                <joint name="fl_axle_{u_id}" type="hinge" axis="0 1 0" damping="1.0" limited="false"/>
                <geom name="fl_wheel_{u_id}" type="cylinder" size="0.40 0.06" euler="-90 0 0" mass="15" friction="0.9 0.005 0.0001" rgba="0.1 0.1 0.1 1"/>
            </body>
        </body>

        <body name="rl_module_{u_id}" pos="-0.60  0.50 0.40">
            <geom name="rl_module_side1_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00  0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <geom name="rl_module_side2_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00 -0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <body name="rl_wheel_body_{u_id}" pos="0 0 0">
                <joint name="rl_axle_{u_id}" type="hinge" axis="0 1 0" damping="1.0" limited="false"/>
                <geom name="rl_wheel_{u_id}" type="cylinder" size="0.40 0.06" euler="-90 0 0" mass="15" friction="0.9 0.005 0.0001" rgba="0.1 0.1 0.1 1"/>
            </body>
        </body>

        <body name="fr_module_{u_id}" pos=" 0.60 -0.50 0.40">
            <geom name="fr_module_side1_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00  0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <geom name="fr_module_side2_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00 -0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <body name="fr_wheel_body_{u_id}" pos="0 0 0">
                <joint name="fr_axle_{u_id}" type="hinge" axis="0 1 0" damping="1.0" limited="false"/>
                <geom name="fr_wheel_{u_id}" type="cylinder" size="0.40 0.06" euler="-90 0 0" mass="15" friction="0.9 0.005 0.0001" rgba="0.1 0.1 0.1 1"/>
            </body>
        </body>

        <body name="rr_module_{u_id}" pos="-0.60 -0.50 0.40">
            <geom name="rr_module_side1_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00  0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <geom name="rr_module_side2_{u_id}" type="box" size="0.20 0.02 0.45" pos="0.00 -0.09 0.45" rgba="0.92 0.92 0.90 1" mass="3"/>
            <body name="rr_wheel_body_{u_id}" pos="0 0 0">
                <joint name="rr_axle_{u_id}" type="hinge" axis="0 1 0" damping="1.0" limited="false"/>
                <geom name="rr_wheel_{u_id}" type="cylinder" size="0.40 0.06" euler="-90 0 0" mass="15" friction="0.9 0.005 0.0001" rgba="0.1 0.1 0.1 1"/>
            </body>
        </body>
    </body>

    <actuators>
        <motor name="fl_motor_{u_id}" joint="fl_axle_{u_id}" gear="1" ctrllimited="true" ctrlrange="-100 100"/>
        <motor name="rl_motor_{u_id}" joint="rl_axle_{u_id}" gear="1" ctrllimited="true" ctrlrange="-100 100"/>
        <motor name="fr_motor_{u_id}" joint="fr_axle_{u_id}" gear="1" ctrllimited="true" ctrlrange="-100 100"/>
        <motor name="rr_motor_{u_id}" joint="rr_axle_{u_id}" gear="1" ctrllimited="true" ctrlrange="-100 100"/>
    </actuators>

    <sensors>
        <accelerometer name="imu_acc_{u_id}" site="imu_site_{u_id}"/>
        <gyro name="imu_gyro_{u_id}" site="imu_site_{u_id}"/>
        <user name="speed_user_{u_id}" dim="1"/>
    </sensors>'''
        
        return complete_xml
    
    def control(self, model, data):
        """Apply PD control to achieve target velocity with torque actuators."""
        if not hasattr(self, 'target_velocities') or not self.target_velocities:
            return
        
        if not hasattr(self, 'instance_id') or not self.instance_id:
            return
        
        fl_target, rl_target, fr_target, rr_target = self.target_velocities
        
        try:
            # Get joint indices.
            fl_joint_id = model.joint(f'fl_axle_{self.instance_id}').id
            rl_joint_id = model.joint(f'rl_axle_{self.instance_id}').id
            fr_joint_id = model.joint(f'fr_axle_{self.instance_id}').id
            rr_joint_id = model.joint(f'rr_axle_{self.instance_id}').id
            
            # Get actuator indices.
            fl_idx = model.actuator(f'fl_motor_{self.instance_id}').id
            rl_idx = model.actuator(f'rl_motor_{self.instance_id}').id
            fr_idx = model.actuator(f'fr_motor_{self.instance_id}').id
            rr_idx = model.actuator(f'rr_motor_{self.instance_id}').id
            
            # Get current velocities.
            fl_vel = data.qvel[fl_joint_id]
            rl_vel = data.qvel[rl_joint_id]
            fr_vel = data.qvel[fr_joint_id]
            rr_vel = data.qvel[rr_joint_id]
            
            # PD control: torque = kp * (target - current) - kd * current.
            # Simple proportional control with velocity damping.
            data.ctrl[fl_idx] = self.kp * (fl_target - fl_vel) - self.kd * fl_vel
            data.ctrl[rl_idx] = self.kp * (rl_target - rl_vel) - self.kd * rl_vel
            data.ctrl[fr_idx] = self.kp * (fr_target - fr_vel) - self.kd * fr_vel
            data.ctrl[rr_idx] = self.kp * (rr_target - rr_vel) - self.kd * rr_vel
            
        except Exception as e:
            pass


class Plant(MujocoBody):
    
    # Scenic property - can be overridden in .scenic files.
    semanticColor: [0, 255, 0]  # Default green.
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.instance_id = None
    
    def semanticColorForGeom(self, geom_name):
        """Return semantic color for a specific geom of the plant.
        
        Args:
            geom_name: Name of the geometry (e.g., "stem_plant_0", "leaf_plant_0")
        
        Returns:
            [R, G, B] color list, or None to use default semanticColor
        """
        # Example: Color stem differently from leaves.
        if 'stem' in geom_name.lower():
            return [139, 69, 19]  # Brown for stem.
        
        if 'leaf' in geom_name.lower():
            return [34, 139, 34]  # Forest green for leaves.
        
        # Default: use the object's semanticColor.
        return None

    def get_mujoco_xml(self, obj_counter, position, quaternion):
        
        u_id = f"plant_{obj_counter}"
        self.instance_id = u_id
        self.body_name = f"frame_{u_id}"
        
        complete_xml = f'''
        <body name="{self.body_name}" pos="{position}">
            <geom name="stem_{u_id}" type="cylinder" size="0.02 0.22" pos="0 0 0.12" rgba="0.22 0.55 0.22 1" contype="1" conaffinity="1"/>
            <geom name="leaf_{u_id}" type="sphere" size="0.05" pos="0 0 0.28" rgba="0.20 0.65 0.20 1" contype="1" conaffinity="1"/>
        </body>'''
        
        return complete_xml