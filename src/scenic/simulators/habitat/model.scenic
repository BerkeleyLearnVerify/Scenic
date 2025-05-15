import math
from scenic.core.utils import repairMesh
from scenic.simulators.habitat.simulator import HabitatSimulation, HabitatSimulator
import trimesh
import habitat.config.default_structured_configs as cfg
from scenic.simulators.habitat.utils import scenic_to_habitat_map, habitat_to_scenic_map
import magnum as mn

simulator HabitatSimulator()
data_dir = '/home/kxu/habitat-lab/data/'

class HabitatAgent():
    name: 'agent'
    object_type: 'agent'
    is_agent: True
    _agent_id: None
    _only_agent: False
    #_object_id: self._articulated_agent.sim_obj.object_id
    _articulated_agent_type: None
    _motion_data_path: None
    _articulated_agent: None
    _policy_path_dict: dict()
    # _sim_sensors: { # TODO temporary
        # "third_rgb": cfg.ThirdRGBSensorConfig(width=1024, height=1024),
        # "head_rgb": cfg.HeadRGBSensorConfig(),
    # }
    _lab_sensors: dict()

    # @property
    # def _action_dict(self):
        # return dict()



class Robot(HabitatAgent):
    name: 'robot'
    object_type: 'robot'
    _articulated_agent_type: None
    urdf_path: ''
    is_agent: True
    position: (0, 0, 0)
    yaw: 0
    roll: 0
    pitch: 0
    _object_template_handle: None
    _has_grasp: True
    _grasp_manager: None
    _ik_arm_urdf: ""
    _ik_helper: None
    _holding_object: False

    def distanceToClosest(self, type: type) -> Object:
        """Compute the distance to the closest object of the given type.

        For example, one could write :scenic:`self.distanceToClosest(Car)` in a behavior.
        """
        objects = simulation().objects
        minDist = float('inf')
        for obj in objects:
            if not isinstance(obj, type):
                continue
            d = distance from self to obj
            if 0 < d < minDist:
                minDist = d
        return minDist

    def getClosest(self, object_class):
        objects = simulation().objects
        minDist = float('inf')
        tgt = None
        for obj in objects:
            if not isinstance(obj, object_class):
                continue
            d = distance from self to obj
            if 0 < d < minDist:
                minDist = d
                tgt = obj
        return tgt

class FetchRobot(Robot):
    name: "FetchRobot"
    object_type: 'FetchRobot'
    _articulated_agent_type: 'FetchRobot'
    # _ik_arm_urdf: "/home/kxu/habitat-lab/data/robots/hab_fetch/robots/fetch_arm.urdf"
    urdf_path: data_dir + 'robots/hab_fetch/robots/hab_fetch.urdf'
    shape: CylinderShape(dimensions=(0.508,0.559,1.096))
    # @property
    # def _action_dict(self):
        # if self._only_agent:
            # return {
                # self.name + "oracle_magic_grasp_action": cfg.ArmActionConfig(type="MagicGraspAction"),
                # self.name + "base_velocity_action": cfg.BaseVelocityActionConfig(),
                # self.name + "oracle_coord_action": cfg.OracleNavActionConfig(type="OracleNavCoordinateAction", 
                                                                              # spawn_max_dist_to_obj=1.0),
                # # self.name + "reach_action": cfg.ArmActionConfig(type="ArmEEAction") # TODO maybe a little more in the setup
            # }

        # return {
            # self.name + "_oracle_magic_grasp_action": cfg.ArmActionConfig(type="MagicGraspAction"),
            # self.name + "_base_velocity_action": cfg.BaseVelocityActionConfig(),
            # self.name + "_oracle_coord_action": cfg.OracleNavActionConfig(type="OracleNavCoordinateAction", 
                                                                          # spawn_max_dist_to_obj=1.0),
            # # self.name + "_reach_action": cfg.ArmActionConfig(type="ArmEEAction") # TODO maybe a little more in the setup
        # }

class SpotRobot(Robot):
    # name: "SpotRobot"
    name: "agent_1"
    object_type: "SpotRobot"
    _articulated_agent_type: "SpotRobot"
    ee_pos: (0, 0.5 ,0)
    urdf_path: data_dir + 'robots/hab_spot_arm/urdf/hab_spot_arm.urdf'
    _policy_path_dict: dict(pick='/home/kxu/Scenic-habitat/src/scenic/simulators/habitat/policies/pick_latest.torchscript',
                       place='/home/kxu/Scenic-habitat/src/scenic/simulators/habitat/policies/place_latest_sample.torchscript')
    _policies: dict()
    shape: BoxShape(dimensions=(1.1, 0.5, 0.7))
    # _sim_sensors: { # TODO temporary
                # "third_rgb": cfg.ThirdRGBSensorConfig(width=1024, height=1024),
                # "head_rgb": cfg.HeadRGBSensorConfig()
                # # "arm_rgb": cfg.ArmRGBSensorConfig()
                # # "articulated_agent_jaw_depth": cfg.JawDepthSensorConfig()
    # }

    # @property
    # def ee_pos(self):
        # ee_pos = self._articulated_agent.ee_transofrm().translation
        # x, y, z, _, _ , _ = habitat_to_scenic_map((ee_pos[0], ee_pos[1], ee_pos[2], 0, 0, 0))
        # return Vector(x, y, z)

    # @property
    # def _action_dict(self):
        # if self._only_agent:
            # return {
                # "oracle_magic_grasp_action": cfg.ArmActionConfig(type="MagicGraspAction"),
                # "base_velocity_action": cfg.BaseVelocityActionConfig(),
                # "oracle_coord_action": cfg.OracleNavActionConfig(type="OracleNavCoordinateAction", 
                                                                              # spawn_max_dist_to_obj=1.0)
            # }

        # return {
            # self.name + "_oracle_magic_grasp_action": cfg.ArmActionConfig(type="MagicGraspAction"),
            # self.name + "_base_velocity_action": cfg.BaseVelocityActionConfig(),
            # self.name + "_oracle_coord_action": cfg.OracleNavActionConfig(type="OracleNavCoordinateAction", 
                                                                          # spawn_max_dist_to_obj=1.0)
        # }



class KinematicHumanoid(HabitatAgent):
    name: "Humanoid"
    object_type: 'KinematicHumanoid'
    _articulated_agent_type: 'KinematicHumanoid'
    _humanoid_controller: None
    _in_position: False
    urdf_path: None
    shape: CylinderShape(dimensions=(0.508,0.559,1.75))

    # @property
    # def _action_dict(self):
        # if self._only_agent:
            # return {
                # "humanoid_joint_action": cfg.HumanoidJointActionConfig(),
                # "humanoid_navigate_action": cfg.OracleNavActionConfig(type="OracleNavCoordinateAction", 
                                                                  # motion_control="human_joints", # name + "_human_joints"???
                                                                  # spawn_max_dist_to_obj=1.0),
                # "humanoid_pick_obj_id_action": cfg.HumanoidPickActionConfig(type="HumanoidPickObjIdAction")}
        # return {
            # self.name + "_humanoid_joint_action": cfg.HumanoidJointActionConfig(),
            # self.name + "_humanoid_navigate_action": cfg.OracleNavActionConfig(type="OracleNavCoordinateAction", 
                                                              # motion_control="human_joints", # name + "_human_joints"???
                                                              # spawn_max_dist_to_obj=1.0),
            # self.name + "_humanoid_pick_obj_id_action": cfg.HumanoidPickActionConfig(type="HumanoidPickObjIdAction")
        # }

    @property
    def ee_pos(self):
        offset =  self._articulated_agent.base_transformation.transform_vector(mn.Vector3(0, 0.3, 0))
        ee_pos = self._articulated_agent.ee_transform(0).translation + offset
        x, y, z, _, _ , _ = habitat_to_scenic_map((ee_pos[0], ee_pos[1], ee_pos[2], 0, 0, 0))
        return Vector(x, y, z)

class Female_0(KinematicHumanoid):
    name: "Female_0"
    urdf_path: data_dir + 'hab3_bench_assets/humanoids/female_0/female_0.urdf'
    _motion_data_path: data_dir + 'hab3_bench_assets/humanoids/female_0/female_0_motion_data_smplx.pkl'


class HabitatObject:
    name: 'HabitatObject'
    object_type: None
    is_agent: False
    _object_id: None
    _use_file_handle: None
    _object_file_handle: None
    _object_template_handle: None
    _managed_rigid_object: None

class MasterChef(HabitatObject):
    name: 'MasterChef'
    object_type: 'MasterChef'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/002_master_chef_can.object_config.json'
    shape: CylinderShape(dimensions=(0.1,0.1,0.5)) # TODO just a dummy dimensions

class TennisBall(HabitatObject):
    name: 'TennisBall'
    object_type: 'TennisBall'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/056_tennis_ball.object_config.json'
    shape: CylinderShape(dimensions=(0.1,0.1,0.5)) # TODO just a dummy dimensions

class GelatinBox(HabitatObject):
    name: 'GelatinBox'
    object_type: 'GelatinBox'
    _use_file_handle: True
    _object_file_handle: data_dir + 'objects/ycb/configs/009_gelatin_box.object_config.json'
    shape: CylinderShape(dimensions=(0.1,0.1,0.08)) # TODO just a dummy dimensions
