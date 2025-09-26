'''
This example is not working, fix: WIP.

'''

# examples/robosuite/four_table_workspace.scenic
model scenic.simulators.robosuite.model

# CAMERA CONFIGURATION
# param camera_view = "sideview"

TABLE_DISTANCE = 1.0

# WORKSPACE: Four tables in cross formation
back_table = new Table at (-TABLE_DISTANCE, 0, 0.425)
front_table = new Table at (TABLE_DISTANCE, 0, 0.425)
right_table = new Table at (0, TABLE_DISTANCE, 0.425)
left_table = new Table at (0, -TABLE_DISTANCE, 0.425)

# TABLE 1 (Back): Two primitive objects with explicit spacing
ball = new Ball at (-TABLE_DISTANCE - 0.2, 0, 0.9),
    with color (1, 0.5, 0, 1)

box = new Box at (-TABLE_DISTANCE + 0.2, 0, 0.9),
    with color (0, 0, 1, 1)

# TABLE 2 (Front): Two nuts
square_nut = new SquareNut at (TABLE_DISTANCE - 0.2, 0, 0.9)

round_nut = new RoundNut at (TABLE_DISTANCE + 0.2, 0, 0.9)

# TABLE 3 (Right): Two food items  
can = new Can at (0, TABLE_DISTANCE - 0.2, 0.9)

bread = new Bread at (0, TABLE_DISTANCE + 0.2, 0.9)

# TABLE 4 (Left): Two more objects
milk = new Milk at (0, -TABLE_DISTANCE - 0.2, 0.9)

bottle = new Bottle at (0, -TABLE_DISTANCE + 0.2, 0.9)

# ROBOT
ego = new Panda at (0, 0, 0)