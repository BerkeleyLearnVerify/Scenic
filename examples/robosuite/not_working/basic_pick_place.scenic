# Basic Pick and Place Test Scenario for RoboSuite
# Tests robot's ability to pick up cubes of varying sizes

model scenic.simulators.robosuite.model

# Parameters
param cube_count = DiscreteRange(3, 6)
param min_cube_size = 0.03  # 3cm
param max_cube_size = 0.08  # 8cm

# Create table
table = new PositionableTable at (0, 0, 0),
    facing 0 deg,
    with width 2,
    with length 1,
    with height 0.8

# Create robot
ego = new PandaRobot at (0, -0.8, 0),
    facing 90 deg

# Create cubes of varying sizes on table
for i in range(globalParameters.cube_count):
    cube_size = Range(globalParameters.min_cube_size, globalParameters.max_cube_size)
    new Cube at (Range(-0.3, 0.3), Range(-0.1, 0.1), table.height + cube_size/2),
        with width cube_size,
        with length cube_size,
        with height cube_size,
        with color (Uniform(0.5, 1), Uniform(0, 0.5), Uniform(0, 0.5))

# Pick and place behavior
behavior PickAndPlaceBehavior():
    # Home position
    take SetJointPositions([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
    for i in range(20):
        wait
    
    # Pre-grasp position
    take SetJointPositions([0.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.08, 0.08])
    for i in range(20):
        wait
    
    # Grasp position
    take SetJointPositions([0.2, -0.3, 0, -1.8, 0, 1.4, 0.785, 0.08, 0.08])
    for i in range(20):
        wait
    
    # Close gripper
    take SetJointPositions([0.2, -0.3, 0, -1.8, 0, 1.4, 0.785, 0.01, 0.01])
    for i in range(10):
        wait
    
    # Lift
    take SetJointPositions([0.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.01, 0.01])
    for i in range(20):
        wait
    
    # Move to place position
    take SetJointPositions([-0.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.01, 0.01])
    for i in range(20):
        wait
    
    # Lower
    take SetJointPositions([-0.2, -0.3, 0, -1.8, 0, 1.4, 0.785, 0.01, 0.01])
    for i in range(20):
        wait
    
    # Open gripper
    take SetJointPositions([-0.2, -0.3, 0, -1.8, 0, 1.4, 0.785, 0.08, 0.08])
    for i in range(10):
        wait
    
    # Retreat
    take SetJointPositions([-0.2, -0.5, 0, -2.0, 0, 1.5, 0.785, 0.08, 0.08])
    for i in range(20):
        wait

# Assign behavior
ego.behavior = PickAndPlaceBehavior()

# Monitor progress
monitor PickPlaceMonitor():
    for i in range(100):
        wait
        if i % 10 == 0:
            print(f"Step {i}: Gripper: {ego.joint_positions[-2]:.3f}, {ego.joint_positions[-1]:.3f}")

# Record data
record ego.joint_positions as robot_joints
record initial globalParameters.cube_count as "total_cubes"

# Simulation parameters
param timestep = 0.1
terminate after 200 steps

require monitor PickPlaceMonitor()