# advanced_actions.scenic
"""Example demonstrating advanced robot actions and property tracking."""
model scenic.simulators.robosuite.model

# Setup arena and robot
arena = TableArena
robot = new PandaRobot at (0, -0.5, 0)

# Objects to manipulate
red_cube = new Cube at (0.2, 0.1, 0.85),
    with color (0.8, 0.2, 0.2)

blue_cube = new Cube at (-0.2, 0.1, 0.85),
    with color (0.2, 0.2, 0.8)

green_ball = new Ball at (0, -0.1, 0.85),
    with color (0.2, 0.8, 0.2)

ego = robot

# Advanced behavior with gripper control
behavior PickAndPlace(robot):
    # Initial position
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
    wait 1
    
    # Open gripper
    take SetGripperState(1.0)
    wait 0.5
    
    # Move to red cube
    take SetJointPositions([0.3, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04])
    wait 2
    
    # Close gripper to grasp
    take SetGripperState(-1.0)
    wait 0.5
    
    # Lift up
    take SetJointPositions([0.3, -0.3, 0, -1.8, 0, 1.3, 0.785, 0.0, 0.0])
    wait 1
    
    # Move to new position
    take SetJointPositions([-0.3, -0.3, 0, -1.8, 0, 1.3, -0.785, 0.0, 0.0])
    wait 2
    
    # Lower down
    take SetJointPositions([-0.3, -0.5, 0, -2.0, 0, 1.5, -0.785, 0.0, 0.0])
    wait 1
    
    # Open gripper to release
    take SetGripperState(1.0)
    wait 0.5
    
    # Move away
    take SetJointPositions([0, -0.5, 0, -2.0, 0, 1.5, 0, 0.04, 0.04])

# Monitor properties
monitor robot.joint_positions
monitor robot.gripper_state
monitor red_cube.position