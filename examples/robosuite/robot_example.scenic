# examples/robosuite/robot_example.scenic
"""Simple robot manipulation example."""

from scenic.simulators.robosuite.model import *

# Create a Panda robot
robot = PandaRobot at (0, 0, 0)

# Create a cube to manipulate
cube = Cube at (0.3, 0, 0.05),
    with color (1, 0, 0)

# Record robot joint positions
record robot.joint_positions as JointPositions

# Simple behavior - move to a specific joint configuration
behavior SimpleMove():
    take SetJointPositions([0, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04])

robot.setAsEgo()
require always robot.joint_positions[0] >= -2.617  # Joint limits
ego = robot with behavior SimpleMove

terminate after 50 steps