"""Robosuite Lift task demonstration in Scenic."""

model scenic.simulators.robosuite.model

arena = new Table at (0, 0, 0)

# arena = new PegsArena

# table = new PositionableTable at (0, 0, 0),
#     facing 0 deg,
#     with width 2,
#     with length 1,
#     with height 0.8

target_cube = new Cube at (0.5, -0.3, 0.825),
    with color (0, 0, 1)

# Create robot positioned next to table, facing it
ego = new PandaRobot at (0, -0.8, 0),
    facing 90 deg  # Face positive Y direction (toward table)

# Create object on table surface (default table top is at z=0.8)
# target_cube = new Cube at (0, 0, 0.825),
#     with width 0.05,
#     with length 0.05,
#     with height 0.05,
#     with color (1, 0, 0)  # Red       

# Define lift behavior
behavior LiftBehavior():
    # Move to pre-grasp position
    take SetJointPositions([0, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.04, 0.04])
    for i in range(20):
        wait
    
    # Open gripper
    take SetJointPositions([0, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.08, 0.08])
    for i in range(10):
        wait
    
    # Move down to object
    take SetJointPositions([0.2, -0.3, 0, -1.8, 0, 1.4, 0.785, 0.08, 0.08])
    for i in range(20):
        wait
    
    # Close gripper
    take SetJointPositions([0.2, -0.3, 0, -1.8, 0, 1.4, 0.785, 0.01, 0.01])
    for i in range(10):
        wait
    
    # Lift up
    take SetJointPositions([0.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.01, 0.01])
    for i in range(20):
        wait

# Assign behavior to ego robot
ego.behavior = LiftBehavior()   

# Monitor to track progress
monitor LiftMonitor():
    for i in range(30):
        wait
        joint_pos = ego.joint_positions
        if joint_pos:
            print(f"Step {i}: Gripper: {joint_pos[-2]:.3f}, {joint_pos[-1]:.3f}")
        print(f"Step {i}: Cube at z={target_cube.position.z:.3f}")

# Record data for visualization
record ego.joint_positions as robot_joints
record ego.position as robot_position
record target_cube.position as cube_position

# Simulation parameters
param timestep = 0.1
terminate after 30 steps

require monitor LiftMonitor()