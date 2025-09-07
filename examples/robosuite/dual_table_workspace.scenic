# examples/robosuite/dual_table_workspace.scenic
model scenic.simulators.robosuite.model

# SCENARIO CONFIGURATION
param use_environment = "Custom"

# WORKSPACE SETUP
back_table = new Table at (-0.6, 0, 0.8)
front_table = new Table at (0.6, 0, 0.8)

# OBJECTS WITH RANDOM PLACEMENT USING RANGE
# Using Range with modern tuple syntax for randomness
red_cube = new Box at (Range(-0.10, -0.15), Range(-0.11, 0.22), 0.83),
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04

green_cube = new Box at (Range(0.5, 0.7), Range(-0.1, 0.1), 0.83),
    with color (0, 1, 0, 1),
    with width 0.05, with length 0.05, with height 0.05

# ROBOT
ego = new Panda at (0, 0, 0)