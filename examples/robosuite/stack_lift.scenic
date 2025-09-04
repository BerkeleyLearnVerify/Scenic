# examples/robosuite/stack_lift.scenic
model scenic.simulators.robosuite.model

# SCENARIO CONFIGURATION
param use_environment = "Custom"
# param camera_view = "sideview"

# WORKSPACE: Single rectangular table positioned in front (rotated 90 degrees)
work_table = new Table at (0.6, 0, 0.8),
    with width 0.6,   # Rotated dimensions
    with length 1.2,
    with height 0.05

# OBJECTS
# Bottom cube (larger, stable base)
bottom_cube = new CustomBox at (0.6, 0, 0.83),
    with envObjectName "cube_bottom",
    with color (0.2, 0.3, 0.8, 1),
    with width 0.06, with length 0.06, with height 0.06

# Top cube (smaller, stacked on bottom)
top_cube = new CustomBox at (0.6, 0, 0.89),  # 0.83 + 0.06 height of bottom
    with envObjectName "cube_top",
    with color (0.8, 0.2, 0.2, 1),
    with width 0.04, with length 0.04, with height 0.04

# Bottle placed separately on table
bottle = new CustomBottle at (0.6, 0.3, 0.83),
    with envObjectName "bottle"

# Custom lift behavior defined locally
behavior CustomLift():
    do PickObject(top_cube)
    do LiftToHeight(1.05)
    
    # Check if cube is above threshold height
    for _ in range(10):  # Check for 10 steps
        if top_cube.position.z > 1.0:
            terminate simulation
        wait

# ROBOT: Using UR5e with custom behavior
ego = new UR5eRobot at (0, 0, 0),
    with behavior CustomLift()