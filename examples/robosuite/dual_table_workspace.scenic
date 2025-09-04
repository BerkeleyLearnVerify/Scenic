# examples/robosuite/dual_table_workspace.scenic
model scenic.simulators.robosuite.model

# SCENARIO CONFIGURATION
param use_environment = "Custom"

# WORKSPACE SETUP
back_table = new Table at (-0.6, 0, 0.8)
front_table = new Table at (0.6, 0, 0.8)

# OBJECTS WITH RANDOM PLACEMENT
red_cube = new CustomBox at (-0.6, 0, 0.82),
    with envObjectName "cube_back",
    with color (1, 0, 0, 1),
    with width 0.04, with length 0.04, with height 0.04,
    with randomPlacement True, with tableIndex 0

green_cube = new CustomBox at (0.6, 0, 0.82),
    with envObjectName "cube_front",
    with color (0, 1, 0, 1),
    with width 0.05, with length 0.05, with height 0.05,
    with randomPlacement True, with tableIndex 1

# ROBOT
ego = new PandaRobot at (0, 0, 0)