import os

from stable_baselines3 import PPO
from scenic.simulators.mujoco.model import *


class Pusher(DynamicMujocoBody):
    """Mujoco Body for Pusher v4 using pretrained Stable Baselines 3 model"""
    def __init__(self, xml: str="", sb3_model: str=None, *args, **kwargs):
        super().__init__(xml, *args, **kwargs)
        root_path = os.getcwd()


        if not sb3_model:
            sb3_model = "PPO_pusher.zip"

        self.controller = PPO.load(os.path.join(root_path, sb3_model), device="cpu")
    
    def control(self, model, data):
        joints = [
            "r_shoulder_pan_joint",
            "r_shoulder_lift_joint",
            "r_upper_arm_roll_joint",
            "r_elbow_flex_joint",
            "r_forearm_roll_joint",
            "r_wrist_flex_joint",
            "r_wrist_roll_joint"
        ]

        joint_positions = []
        joint_velocities = []



        for joint in joints:
            full_joint_name = self.body_name + joint
            joint_positions.append(data.joint(full_joint_name).qpos[0])
            joint_velocities.append(data.joint(full_joint_name).qvel[0])
        
        observation = joint_positions + joint_velocities + [0 for i in range(9)]
        ctrl = self.controller.predict(observation)

        for i, joint in enumerate(joints):
            full_joint_name = self.body_name + "/" + f"unnamed_actuator_{i}"

            actuator = data.actuator(full_joint_name)
        
            actuator.ctrl = ctrl[0][i]

