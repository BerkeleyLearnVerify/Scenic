# examples/robosuite/dual_table_workspace.scenic
model scenic.simulators.robosuite.model

# Arena Setup
back_table = new Table at (-0.6, 0, 0.8)
front_table = new Table at (0.6, 0, 0.8)

# OBJECTS 
red_cube = new Box at (Range(-0.7, -0.5), Range(-0.1, 0.1), 0.83),
    with color (1, 0, 0, 1),
    

green_cube = new Box at (Range(0.5, 0.7), Range(-0.1, 0.1), 0.83),
    with color (0, 1, 0, 1),
    with width 0.05, with length 0.05, with height 0.05

# ROBOT
ego = new Panda at (0, 0, 0)