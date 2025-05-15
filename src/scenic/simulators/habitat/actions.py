import habitat_sim
import magnum as mn
import warnings
from habitat.tasks.rearrange.rearrange_sim import RearrangeSim
warnings.filterwarnings('ignore')
from habitat_sim.utils.settings import make_cfg
from matplotlib import pyplot as plt
from habitat_sim.utils import viz_utils as vut
from omegaconf import DictConfig
import numpy as np
from habitat.articulated_agents.robots import FetchRobot
from habitat.config.default import get_agent_config
from habitat.config.default_structured_configs import ThirdRGBSensorConfig, HeadRGBSensorConfig, HeadPanopticSensorConfig
from habitat.config.default_structured_configs import SimulatorConfig, HabitatSimV0Config, AgentConfig
from habitat.config.default import get_agent_config
from habitat.tasks.rearrange.actions.actions import HumanoidJointAction
import habitat
from habitat_sim.physics import JointMotorSettings, MotionType
from omegaconf import OmegaConf

from scenic.core.simulators import *
from scenic.simulators.habitat.utils import scenic_to_habitat_map


class GoRelDeltaAction(Action):

    def __init__(self, dx=0, dy=0, dz=0, rot=0):
        # print(f"action dx = {dx}")
        # print(f"action goal = {dx, dy, dz}")
        self.pos_delta = mn.Vector3(dx, dy, dz)
        # self.art_agent = obj._articulated_agent
        #TODO add rotation delta
    
    def applyTo(self, obj, sim):
        self.art_agent = obj._articulated_agent
        x, y, z = self.pos_delta
        x, y, z, _, _, _ = sim.scenicToHabitatMap((x, y, z,0,0,0))
        self.pos_delta = np.array([x, y, z])
        self.art_agent.base_pos = self.art_agent.base_pos + self.pos_delta
        return

class RotDeltaAction(Action):
    def __init__(self, rot_delta):
        self.rot_delta = rot_delta

    def applyTo(self, obj, sim):
        obj._articulated_agent.base_rot += self.rot_delta

class HumanGoAction(Action):
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        self.art_agent = obj._articulated_agent
        x, y, z, _, _, _ = sim.scenicToHabitatMap((self.x, self.y, self.z,0,0,0)) # TODO some sketchy coordinate transfomr here

        rel_pose = mn.Vector3(x, y, z)

        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        obj._humanoid_controller.calculate_walk_pose(rel_pose)

        human_joints_trans = obj._humanoid_controller.get_pose()
        
        arg_name = obj._humanoid_joint_action._action_arg_prefix + "human_joints_trans"
        arg_dict = {arg_name: human_joints_trans}
        obj._humanoid_joint_action.step(**arg_dict)

class HumanGoEnvAction(Action):
    """
    This works, yay!
    """
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        self.art_agent = obj._articulated_agent
        x, y, z, _, _, _ = sim.scenicToHabitatMap((self.x, self.y, self.z,0,0,0)) # TODO some sketchy coordinate transfomr here

        rel_pose = mn.Vector3(x, y, z)

        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        obj._humanoid_controller.calculate_walk_pose(rel_pose)

        human_joints_trans = obj._humanoid_controller.get_pose()
        
        sim.step_action_dict["action"] += tuple([obj.name + "_humanoid_joint_action"])
        sim.step_action_dict["action_args"][obj.name + "_human_joints_trans"] = human_joints_trans

class HumanStopAction(Action):
    def applyTo(self, obj, sim):
        self.art_agent = obj._articulated_agent

        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        obj._humanoid_controller.calculate_stop_pose()

        human_joints_trans = obj._humanoid_controller.get_pose()
        
        arg_name = obj._humanoid_joint_action._action_arg_prefix + "human_joints_trans"
        arg_dict = {arg_name: human_joints_trans}
        obj._humanoid_joint_action.step(**arg_dict)


class HumanReachAction(Action):
    """
    Still in the works
    target_pos is the relative position
    seems like z is actually up
    """
    def __init__(self, x=0, y=0, z=0, index_hand=0):
        self.x = x
        self.y = y
        self.z = z
        self.index_hand = index_hand

    def applyTo(self, obj, sim):
        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        offset = obj._articulated_agent.base_transformation.transform_vector(mn.Vector3(0, 0.3, 0)) # offset for hand
        hand_pose = obj._articulated_agent.ee_transform(self.index_hand).translation + offset
        x, y, z, _, _, _ = sim.scenicToHabitatMap((self.x, self.y, self.z, 0, 0, 0))
        hand_pose = hand_pose + mn.Vector3(x, y, z)
        obj._humanoid_controller.calculate_reach_pose(hand_pose, index_hand=self.index_hand)

        new_pose = obj._humanoid_controller.get_pose()
        
        sim.step_action_dict["action"] += tuple([obj.name + "_humanoid_joint_action"])
        sim.step_action_dict["action_args"][obj.name + "_human_joints_trans"] = new_pose
        

# class HumanReachNeutralAction(Action):
    # def applyTo(self, obj, sim):
        # pose =  mn.Vector(-4.86974, 0.731405, 0.0187876)

class HumanReachAbsAction(Action):
    def __init__(self, x=0, y=0, z=0, index_hand=0):
        self.x = x
        self.y = y
        self.z = z
        self.index_hand = index_hand

    def applyTo(self, obj, sim):
        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        # offset = obj._articulated_agent.base_transformation.transform_vector(mn.Vector3(0, 0.3, 0)) # offset for hand
        # hand_pose = obj._articulated_agent.ee_transform(self.index_hand).translation + offset
        x, y, z, _, _, _ = sim.scenicToHabitatMap((self.x, self.y, self.z, 0, 0, 0))
        hand_pose = mn.Vector3(x, y, z)
        obj._humanoid_controller.calculate_reach_pose(hand_pose, index_hand=self.index_hand)

        new_pose = obj._humanoid_controller.get_pose()
        
        sim.step_action_dict["action"] += tuple([obj.name + "_humanoid_joint_action"])
        sim.step_action_dict["action_args"][obj.name + "_human_joints_trans"] = new_pose


class HumanoidNavAction(Action):
    """
    Carry out navigating to an object for one timestep
    """
    def __init__(self, x=0, y=0, z=0):
        """
        Vector obj_pos: object position in Habitat coordinates
        """
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        # print(f"HUMANOID NAVING")

        obj._humanoid_controller.reset(obj._articulated_agent.base_transformation) # probelm, likely relative to human frame?
        x, y, z, _, _, _ = scenic_to_habitat_map((self.x, self.y, self.z, 0, 0, 0))
        object_trans = mn.Vector3(x, y, z)
        if not obj._only_agent:
            sim.step_action_dict["action"] += tuple([obj.name + "_humanoid_navigate_action"])
            sim.step_action_dict["action_args"][obj.name + "_oracle_nav_lookat_action"] = object_trans
            sim.step_action_dict["action_args"][obj.name + "_mode"] = 1  # not sure what this means, but it is done in the tutorial
        else:
            sim.step_action_dict["action"] += tuple([obj.name + "humanoid_navigate_action"])
            sim.step_action_dict["action_args"][obj.name + "oracle_nav_lookat_action"] = object_trans
            sim.step_action_dict["action_args"][obj.name + "mode"] = 1  # not sure what this means, but it is done in the tutorial

class DummyHumanNav(Action):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        object_trans = mn.Vector3(self.x, self.y, self.z)
        # print("Target POS: ", object_trans)
        # print("OBJ NAME: ", obj.name)
        sim.step_action_dict["action"] += tuple([obj.name + "_humanoid_navigate_action"])
        sim.step_action_dict["action_args"][obj.name + "_oracle_nav_lookat_action"] = object_trans
        sim.step_action_dict["action_args"][obj.name + "_mode"] = 1  # not sure what this means, but it is done in the tutorial
        
        

class OracleCoordAction(Action):

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        x, y, z, _, _, _ = scenic_to_habitat_map((self.x, self.y, self.z, 0, 0, 0))
        # object_trans = mn.Vector3(x, y, z)
        object_trans = np.array([x, y, z])
        # print(f"OBJ NAME: {obj.object_type}")
        sim.step_action_dict["action"] += tuple([obj.name + "_oracle_coord_action"])
        sim.step_action_dict["action_args"][obj.name + "_oracle_nav_lookat_action"] = object_trans

class OracleMagicGraspAction(Action):
    def __init__(self, grip_action=0):
        self.grip_action = grip_action

    def applyTo(self, obj, sim):
        sim.step_action_dict["action"] += tuple([obj.name + "_oracle_magic_grasp_action"])
        # sim.step_action_dict["action_args"][obj.name + "_oracle_magic_grasp_action"] = self.grip_action
        # sim.step_action_dict["action_args"][obj.name + "_grip_action"] = self.grip_action
        sim.step_action_dict["action_args"]["grip_action"] = self.grip_action


class OpenGripperAction(Action):
    def applyTo(self, obj, sim):
        obj._articulated_agent.open_gripper()

class CloseGripperAction(Action):
    def applyTo(self, obj, sim):
        obj._articulated_agent.close_gripper()

class SnapToObjectAction(Action):
    def __init__(self, target_obj):
        self.target_obj_id = target_obj._object_id

    def applyTo(self, obj, sim):
        obj._grasp_manager.snap_to_obj(self.target_obj_id)
        obj._holding_object = True


class SpotMoveArmAction(Action):
    def __init__(self, arm_ctrl_angles=[0.0, -3.14, 0.0, 3.0, 0.0, 0.0, 0.0]):
        self.arm_ctrl_angles = arm_ctrl_angles

    def applyTo(self, obj, sim):

        # arm_ctrl = [0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0]
        obj._articulated_agent.arm_joint_pos = self.arm_ctrl_angles

class FetchReachAction(Action):
    def __init__(self, x=0, y=0, z=0, frame='world'):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        x, y, z, _, _, _ = scenic_to_habitat_map((self.x, self.y, self.z, 0, 0, 0))
        # print(f"TARGET EE POINT WORLD FRAME: {x, y, z}")
        transformation = obj._articulated_agent.sim_obj.transformation
        transformation_inv = transformation.inverted()
        ee_pos = transformation_inv.transform_point(mn.Vector3(x, y, z))
        # ee_pos = transformation.transform_point(mn.Vector3(x, y, z))
        # print(f"TRANSFORMED EE POINT BASE FRAME: {ee_pos}")
        # TODO can reference the steps done for the HumanReachAction
        # transform = obj._articulated_agent.ee_transform()
        # ee_pos = mn.Vector3(x, y, z)
        # print(f"TARGET EE POS: {ee_pos}")
        # ee_pos = transform.transform_vector(ee_pos)

        sim.step_action_dict["action"] += tuple([obj.name + "_reach_action"])
        sim.step_action_dict["action_args"]["ee_pos"] = ee_pos

class FetchSetJointIKAction(Action):
    def __init__(self, x=0, y=0, z=0, frame='world'):
        self.x = x
        self.y = y
        self.z = z

    def applyTo(self, obj, sim):
        x, y, z, _, _, _ = scenic_to_habitat_map((self.x, self.y, self.z, 0, 0, 0))
        # print(f"TARGET EE POINT WORLD FRAME: {x, y, z}")
        transformation = obj._articulated_agent.sim_obj.transformation
        transformation_inv = transformation.inverted()
        ee_pos = transformation_inv.transform_point(mn.Vector3(x, y, z))
        joint_pos = obj._ik_helper.calc_ik(np.array([x,y,z]))
        # obj._articulated_agent.arm_joint_pos = joint_pos
        # est_pos = obj._ik_helper.calc_fk(np.array(joint_pos))
        # print(f"Joint Pos resulting pos{est_pos}")
        obj._articulated_agent.arm_motor_pos = list(joint_pos)

class FetchSetJointAction(Action):
    def __init__(self, joint_state):
        self.joint_state = joint_state

    def applyTo(self, obj, sim):
        obj._articulated_agent.arm_joint_pos = self.joint_state

