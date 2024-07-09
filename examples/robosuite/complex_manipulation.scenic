# complex_manipulation.scenic
"""Example of complex manipulation task with full action system."""
model scenic.simulators.robosuite.model

# Multi-table setup
table1 = new PositionableTable at (-0.6, 0, 0),
    with width 0.8,
    with height 0.8

table2 = new PositionableTable at (0.6, 0, 0),
    with width 0.8,
    with height 0.8

robot = new PandaRobot at (0, -0.5, 0)

# Objects to sort
red_cube = new Cube at (-0.6, 0, 0.85),
    with color (1.0, 0.0, 0.0)

blue_cube = new Cube at (-0.5, 0.1, 0.85),
    with color (0.0, 0.0, 1.0)

green_cube = new Cube at (-0.7, 0.1, 0.85),
    with color (0.0, 1.0, 0.0)

ego = robot

# Complex sorting behavior
behavior SortByColor(robot):
    # Initialize
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
    take SetGripperState(1.0)  # Open
    wait 1
    
    # Pick up red cube
    take SetJointPositions([-0.8, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04])
    wait 1.5
    take SetGripperState(-1.0)  # Close
    wait 0.5
    
    # Lift
    take SetJointPositions([-0.8, -0.3, 0, -1.8, 0, 1.3, 0.785, 0.0, 0.0])
    wait 1
    
    # Move to table 2
    take SetJointPositions([0.8, -0.3, 0, -1.8, 0, 1.3, -0.785, 0.0, 0.0])
    wait 2
    
    # Place
    take SetJointPositions([0.8, -0.5, 0, -2.0, 0, 1.5, -0.785, 0.0, 0.0])
    wait 1
    take SetGripperState(1.0)  # Open
    wait 0.5
    
    # Return for blue cube
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
    wait 2
    
    # Pick up blue cube
    take SetJointPositions([-0.6, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04])
    wait 1.5
    take SetGripperState(-1.0)
    wait 0.5
    
    # Lift and move
    take SetJointPositions([-0.6, -0.3, 0, -1.8, 0, 1.3, 0.785, 0.0, 0.0])
    wait 1
    take SetJointPositions([0.6, -0.3, 0, -1.8, 0, 1.3, -0.785, 0.0, 0.0])
    wait 2
    
    # Place next to red
    take SetJointPositions([0.6, -0.5, 0, -2.0, 0, 1.5, -0.785, 0.0, 0.0])
    wait 1
    take SetGripperState(1.0)
    
    # Final position
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])

# Track all movements
monitor robot.joint_positions
monitor robot.gripper_state
monitor red_cube.position
monitor blue_cube.position
monitor green_cube.position