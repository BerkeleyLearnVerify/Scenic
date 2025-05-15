from habitat.articulated_agent_controllers import HumanoidRearrangeController, HumanoidSeqPoseController
from habitat.config.default_structured_configs import HumanoidJointActionConfig, HumanoidPickActionConfig
from habitat.tasks.rearrange.actions.actions import HumanoidJointAction

def walk(x, y, z, obj, sim):
    obj._humanoid_controller = HumanoidRearrangeController(obj._motion_data_path)
    obj._humanoid_joint_action = HumanoidJointAction(config=HumanoidJointActionConfig(),
                                                     sim=obj.sim, name=f'agent_{obj._agent_id}')

    rel_pose = mn.Vector3(x, y, z)
    obj._humanoid_controller.calculate_walk_pose(rel_pose) 
    human_joint_trans = obj._humanoid_controller.get_pose()

    arg_name = obj._humanoid_joint_action._action_arg_prefix + "human_joints_trans"
    arg_dict = {arg_name: human_joints_trans}
    obj._humanoid_joint_action.step(**arg_dict)
