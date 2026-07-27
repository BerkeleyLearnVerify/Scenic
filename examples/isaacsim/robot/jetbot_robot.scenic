model scenic.simulators.isaac.model

# takes a command and returns an ArticulationAction
# https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_adding_controller.html
def jetbotControl(command):

    from scenic.simulators.isaac.backends import articulationAction
    throttle, steering = command
    wheelRadius = 0.03
    wheelBase = 0.1125
    joint_indices = np.array([0, 1])

    joint_velocities = [0.0, 0.0]
    joint_velocities[0] = ((2 * throttle) - (steering * wheelBase)) / (2 * wheelRadius)
    joint_velocities[1] = ((2 * throttle) + (steering * wheelBase)) / (2 * wheelRadius)

    return articulationAction(joint_velocities=joint_velocities, joint_indices=joint_indices)

# Jetbot robot
class Jetbot(IsaacSimRobot):
    width: 0.16
    length: 0.16
    height: 0.12
    isaacAssetPath: "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    control: jetbotControl
