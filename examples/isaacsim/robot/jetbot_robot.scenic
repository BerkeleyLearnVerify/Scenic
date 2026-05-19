model scenic.simulators.isaac.model

# takes a command and returns an ArticulationAction
# https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_controller.html
def jetbot_control(command):

    from scenic.simulators.isaac.backends import articulation_action
    throttle, steering = command
    wheel_radius = 0.03 
    wheel_base = 0.1125
    joint_indices = np.array([0, 1]) 

    joint_velocities = [0.0, 0.0]
    joint_velocities[0] = ((2 * throttle) - (steering * wheel_base)) / (2 * wheel_radius)
    joint_velocities[1] = ((2 * throttle) + (steering * wheel_base)) / (2 * wheel_radius)

    return articulation_action(joint_velocities=joint_velocities, joint_indices=joint_indices)

# Jetbot robot
class Jetbot(IsaacSimRobot):
    width: 0.16
    length: 0.16
    height: 0.12
    isaac_asset_path: "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    control: jetbot_control
