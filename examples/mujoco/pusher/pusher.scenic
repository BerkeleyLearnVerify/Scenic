import os
import numpy as np
from stable_baselines3 import PPO
from scenic.simulators.mujoco.model import DynamicMujocoBody


class Pusher(DynamicMujocoBody):
    """Mujoco Body for Pusher v4 using pretrained Stable Baselines 3 model"""
    
    def __init__(self, *args, **kwargs):
        # Extract sb3_model from kwargs if present - can be full path or just filename
        self.sb3_model_path = kwargs.pop('sb3_model', "./PPO_pusher_improved.zip")
        
        super().__init__(*args, **kwargs)
        
        self.controller = None  # Will be loaded lazily
        self.instance_id = None  # Will be set by simulator

    def get_mujoco_xml(self, obj_counter, position, quaternion):
        """Generate MuJoCo XML for this Pusher instance - single XML block approach."""
        
        u_id = f"pusher_{obj_counter}"
        self.instance_id = u_id
        self.body_name = f"r_shoulder_pan_link_{u_id}"  # Main arm body instead of frame
        
        # Complete XML in one natural block - matches original pusher.xml structure
        complete_xml = f'''
    <!-- Object to push - separate top-level body -->
    <body name="object_{u_id}" pos="0.45 -0.05 -0.275" >
        <geom name="obj_collision_{u_id}" rgba="1 1 1 0" type="sphere" size="0.05" density="100" contype="1" conaffinity="2" friction="0.8 0.1 0.1"/>
        <geom name="obj_visual_{u_id}" rgba="0 1 0 1" type="cylinder" size="0.05 0.05" density="0.00001" contype="0" conaffinity="0"/>
        <joint name="obj_slidey_{u_id}" type="slide" pos="0 0 0" axis="0 1 0" range="-10.3213 10.3" damping="0.5"/>
        <joint name="obj_slidex_{u_id}" type="slide" pos="0 0 0" axis="1 0 0" range="-10.3213 10.3" damping="0.5"/>
    </body>

    <!-- Goal visualization - separate top-level body -->
    <body name="goal_{u_id}" pos="0.2 0.3 -0.275">
        <geom name="goal_visual_{u_id}" rgba="1 0 0 0.5" type="cylinder" size="0.08 0.001" density='0.00001' contype="0" conaffinity="0"/>
    </body>

    <!-- Pusher arm assembly - main body with mass -->
    <body name="r_shoulder_pan_link_{u_id}" pos="0 -0.6 0">
        <!-- Shoulder visual elements -->
        <geom name="e1_{u_id}" type="sphere" rgba="0.6 0.6 0.6 1" pos="-0.06 0.05 0.2" size="0.05" contype="0" conaffinity="0"/>
        <geom name="e2_{u_id}" type="sphere" rgba="0.6 0.6 0.6 1" pos=" 0.06 0.05 0.2" size="0.05" contype="0" conaffinity="0"/>
        <geom name="e1p_{u_id}" type="sphere" rgba="0.1 0.1 0.1 1" pos="-0.06 0.09 0.2" size="0.03" contype="0" conaffinity="0"/>
        <geom name="e2p_{u_id}" type="sphere" rgba="0.1 0.1 0.1 1" pos=" 0.06 0.09 0.2" size="0.03" contype="0" conaffinity="0"/>
        
        <geom name="sp_{u_id}" type="capsule" fromto="0 0 -0.4 0 0 0.2" size="0.1" contype="2" conaffinity="1"/>
        <joint name="r_shoulder_pan_joint_{u_id}" type="hinge" pos="0 0 0" axis="0 0 1" range="-2.2854 1.714602" damping="1.0" />

        <body name="r_shoulder_lift_link_{u_id}" pos="0.1 0 0">
            <geom name="sl_{u_id}" type="capsule" fromto="0 -0.1 0 0 0.1 0" size="0.1" contype="2" conaffinity="1"/>
            <joint name="r_shoulder_lift_joint_{u_id}" type="hinge" pos="0 0 0" axis="0 1 0" range="-0.5236 1.3963" damping="1.0" />

            <body name="r_upper_arm_roll_link_{u_id}" pos="0 0 0">
                <geom name="uar_{u_id}" type="capsule" fromto="-0.1 0 0 0.1 0 0" size="0.02" contype="2" conaffinity="1"/>
                <joint name="r_upper_arm_roll_joint_{u_id}" type="hinge" pos="0 0 0" axis="1 0 0" range="-1.5 1.7" damping="0.1" />

                <body name="r_upper_arm_link_{u_id}" pos="0 0 0">
                    <geom name="ua_{u_id}" type="capsule" fromto="0 0 0 0.4 0 0" size="0.06" contype="2" conaffinity="1"/>

                    <body name="r_elbow_flex_link_{u_id}" pos="0.4 0 0">
                        <geom name="ef_{u_id}" type="capsule" fromto="0 -0.02 0 0.0 0.02 0" size="0.06" contype="2" conaffinity="1"/>
                        <joint name="r_elbow_flex_joint_{u_id}" type="hinge" pos="0 0 0" axis="0 1 0" range="-2.3213 0" damping="0.1" />

                        <body name="r_forearm_roll_link_{u_id}" pos="0 0 0">
                            <geom name="fr_{u_id}" type="capsule" fromto="-0.1 0 0 0.1 0 0" size="0.02" contype="2" conaffinity="1"/>
                            <joint name="r_forearm_roll_joint_{u_id}" type="hinge" limited="true" pos="0 0 0" axis="1 0 0" damping=".1" range="-1.5 1.5"/>

                            <body name="r_forearm_link_{u_id}" pos="0 0 0">
                                <geom name="fa_{u_id}" type="capsule" fromto="0 0 0 0.291 0 0" size="0.05" contype="2" conaffinity="1"/>

                                <body name="r_wrist_flex_link_{u_id}" pos="0.321 0 0">
                                    <geom name="wf_{u_id}" type="capsule" fromto="0 -0.02 0 0 0.02 0" size="0.01" contype="2" conaffinity="1"/>
                                    <joint name="r_wrist_flex_joint_{u_id}" type="hinge" pos="0 0 0" axis="0 1 0" range="-1.094 0" damping=".1" />

                                    <body name="r_wrist_roll_link_{u_id}" pos="0 0 0">
                                        <joint name="r_wrist_roll_joint_{u_id}" type="hinge" pos="0 0 0" limited="true" axis="1 0 0" damping="0.1" range="-1.5 1.5"/>
                                        
                                        <body name="tips_arm_{u_id}" pos="0 0 0">
                                            <geom name="tip_arml_{u_id}" type="sphere" pos="0.1 -0.1 0." size="0.01" contype="2" conaffinity="1"/>
                                            <geom name="tip_armr_{u_id}" type="sphere" pos="0.1 0.1 0." size="0.01" contype="2" conaffinity="1"/>
                                        </body>
                                        
                                        <!-- End effector collision geometry -->
                                        <geom name="ee_main_{u_id}" type="capsule" fromto="0 -0.1 0. 0.0 +0.1 0" size="0.02" contype="2" conaffinity="1"/>
                                        <geom name="ee_left_{u_id}" type="capsule" fromto="0 -0.1 0. 0.1 -0.1 0" size="0.02" contype="2" conaffinity="1"/>
                                        <geom name="ee_right_{u_id}" type="capsule" fromto="0 +0.1 0. 0.1 +0.1 0." size="0.02" contype="2" conaffinity="1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </body>

    <actuators>
        <motor name="motor_shoulder_pan_{u_id}" joint="r_shoulder_pan_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="100"/>
        <motor name="motor_shoulder_lift_{u_id}" joint="r_shoulder_lift_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="100"/>
        <motor name="motor_upper_arm_roll_{u_id}" joint="r_upper_arm_roll_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="100"/>
        <motor name="motor_elbow_flex_{u_id}" joint="r_elbow_flex_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="100"/>
        <motor name="motor_forearm_roll_{u_id}" joint="r_forearm_roll_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="50"/>
        <motor name="motor_wrist_flex_{u_id}" joint="r_wrist_flex_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="50"/>
        <motor name="motor_wrist_roll_{u_id}" joint="r_wrist_roll_joint_{u_id}" ctrlrange="-2.0 2.0" ctrllimited="true" gear="50"/>
    </actuators>

    <sensors>
        <!-- Add any sensors here if needed -->
    </sensors>'''
        
        return complete_xml
    
    def control(self, model, data):
        """Control using Stable Baselines 3 model"""
        if not self.controller:
            try:
                print(f"Loading SB3 model from: {self.sb3_model_path}")
                self.controller = PPO.load(self.sb3_model_path, device="cpu")
                print("SB3 model loaded successfully!")
            except FileNotFoundError:
                print(f"ERROR: SB3 model not found at: {self.sb3_model_path}")
                print("Please check the path and ensure the model file exists.")
                return
            except Exception as e:
                print(f"ERROR: Failed to load SB3 model: {e}")
                print(f"Model path: {self.sb3_model_path}")
                return
        
        if not self.instance_id:
            return  # Not ready yet
        
        try:
            # Create proper observation matching Pusher-v5 format (23 dimensions)
            observation = self._get_observation(model, data)
            
            # Get control commands from SB3 model
            action, _ = self.controller.predict(observation, deterministic=True)
            
            # Motor names with instance_id
            motor_names = [
                f"motor_shoulder_pan_{self.instance_id}",
                f"motor_shoulder_lift_{self.instance_id}",
                f"motor_upper_arm_roll_{self.instance_id}",
                f"motor_elbow_flex_{self.instance_id}",
                f"motor_forearm_roll_{self.instance_id}",
                f"motor_wrist_flex_{self.instance_id}",
                f"motor_wrist_roll_{self.instance_id}"
            ]
            
            # Apply control commands to actuators with proper scaling
            for i, motor_name in enumerate(motor_names):
                if i < len(action):
                    try:
                        motor_idx = model.actuator(motor_name).id
                        # Scale action like in the working bridge version
                        scaled_action = np.clip(action[i] * 0.08, -0.4, 0.4)
                        data.ctrl[motor_idx] = scaled_action
                    except:
                        # Silently fail during early simulation steps
                        pass
                        
        except Exception as e:
            # Silently handle errors during early simulation steps
            pass

    def _get_observation(self, model, data):
        """Create observation vector matching Gym Pusher-v5 environment (23 dimensions)"""
        
        # Joint names
        joint_names = [
            f"r_shoulder_pan_joint_{self.instance_id}",
            f"r_shoulder_lift_joint_{self.instance_id}",
            f"r_upper_arm_roll_joint_{self.instance_id}",
            f"r_elbow_flex_joint_{self.instance_id}",
            f"r_forearm_roll_joint_{self.instance_id}",
            f"r_wrist_flex_joint_{self.instance_id}",
            f"r_wrist_roll_joint_{self.instance_id}"
        ]
        
        joint_positions = []
        joint_velocities = []
        
        # Get joint states
        for joint_name in joint_names:
            try:
                joint_data = data.joint(joint_name)
                joint_positions.append(joint_data.qpos[0] if len(joint_data.qpos) > 0 else 0.0)
                joint_velocities.append(joint_data.qvel[0] if len(joint_data.qvel) > 0 else 0.0)
            except:
                joint_positions.append(0.0)
                joint_velocities.append(0.0)
        
        # Ensure exactly 7 joint values
        joint_positions = np.array(joint_positions[:7] + [0.0] * (7 - len(joint_positions)))
        joint_velocities = np.array(joint_velocities[:7] + [0.0] * (7 - len(joint_velocities)))
        
        # Get fingertip position (3D: x, y, z)
        try:
            fingertip_id = model.body(f"tips_arm_{self.instance_id}").id
            fingertip_pos = data.xpos[fingertip_id][:3]
        except:
            try:
                fingertip_id = model.body(f"r_wrist_roll_link_{self.instance_id}").id
                fingertip_pos = data.xpos[fingertip_id][:3]
            except:
                fingertip_pos = np.array([0.0, 0.0, 0.0])
        
        # Get object position (3D: x, y, z)
        try:
            obj_id = model.body(f"object_{self.instance_id}").id
            obj_pos = data.xpos[obj_id][:3]
        except:
            obj_pos = np.array([0.0, 0.0, 0.0])
        
        # Get goal position (3D: x, y, z)
        try:
            goal_id = model.body(f"goal_{self.instance_id}").id
            goal_pos = data.xpos[goal_id][:3]
        except:
            goal_pos = np.array([0.0, 0.0, 0.0])
        
        # Construct observation vector: [joint_pos(7), joint_vel(7), fingertip_pos(3), obj_pos(3), goal_pos(3)]
        observation = np.concatenate([
            joint_positions.astype(np.float32),
            joint_velocities.astype(np.float32),
            fingertip_pos.astype(np.float32),
            obj_pos.astype(np.float32),
            goal_pos.astype(np.float32)
        ])
        
        return observation