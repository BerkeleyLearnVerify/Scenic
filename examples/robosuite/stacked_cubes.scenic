# examples/robosuite/stacked_cubes.scenic
model scenic.simulators.robosuite.model

# SCENARIO CONFIGURATION
param use_environment = "Custom"
param camera_view = "sideview"

# WORKSPACE: Single rectangular table positioned in front
work_table = new Table at (0.6, 0, 0.8),
    with width 0.6,
    with length 1.2,
    with height 0.05

# OBJECTS
# Bottom cube (larger, stable base)
bottom_cube = new Box at (0.6, 0, 0.83),
    with color (0.2, 0.3, 0.8, 1),
    with width 0.06, with length 0.06, with height 0.06

# Top cube (smaller, stacked on bottom)
top_cube = new Box at (0.6, 0, 0.89),  # 0.83 + 0.06 height of bottom
    with color (0.8, 0.2, 0.2, 1),
    with width 0.04, with length 0.04, with height 0.04

# Bottle placed separately on table
bottle = new Bottle at (0.6, 0.3, 0.83)

# ROBOT: Using UR5e with simplified naming
ego = new UR5e at (0, 0, 0)