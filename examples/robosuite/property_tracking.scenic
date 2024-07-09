# property_tracking.scenic
"""Example demonstrating property tracking capabilities."""
model scenic.simulators.robosuite.model

# Setup
arena = TableArena
robot = new PandaRobot at (0, -0.5, 0)

# Moving object
ball = new Ball at (0, 0, 0.85),
    with color (1.0, 0.5, 0.0)  # Orange

# Static reference
marker = new Cube at (0.3, 0, 0.85),
    with width 0.02,
    with length 0.02,
    with height 0.02,
    with color (0.0, 0.0, 0.0)  # Black

ego = robot

# Behavior that moves robot and tracks properties
behavior TrackMovement(robot):
    # Record initial state
    record robot.position as initial_robot_pos
    record robot.joint_positions as initial_joints
    record robot.end_effector_position as initial_ee_pos
    record ball.position as initial_ball_pos
    
    # Move robot
    take SetJointPositions([0.5, -0.5, 0, -2.0, 0, 1.5, 0.5, 0.04, 0.04])
    wait 2
    
    # Record after movement
    record robot.position as final_robot_pos
    record robot.joint_positions as final_joints
    record robot.end_effector_position as final_ee_pos
    record robot.gripper_state as gripper_status
    
    # Try different joint velocities (if supported)
    take SetJointVelocities([0.1, -0.1, 0, 0.1, 0, -0.1, 0.1, 0, 0])
    wait 1
    
    record robot.joint_velocities as current_velocities
    
    # Return to initial position
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])

# Monitor key properties
monitor robot.joint_positions
monitor robot.end_effector_position
monitor ball.position
monitor ball.velocity