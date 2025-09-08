# examples/robosuite/four_table_workspace.scenic
model scenic.simulators.robosuite.model

# CAMERA CONFIGURATION
# param camera_view = "sideview"

TABLE_DISTANCE = 1.0

# WORKSPACE: Four tables in cross formation
back_table = new Table at (-TABLE_DISTANCE, 0, 0.8)
front_table = new Table at (TABLE_DISTANCE, 0, 0.8)
right_table = new Table at (0, TABLE_DISTANCE, 0.8)
left_table = new Table at (0, -TABLE_DISTANCE, 0.8)

# TABLE 1 (Back): Primitive objects with random positions
ball = new Ball at (Range(-1.1, -0.9), Range(-0.1, 0.1), 0.83),
    with color (1, 0.5, 0, 1)

box = new Box at (Range(-1.1, -0.9), Range(-0.1, 0.1), 0.83),
    with color (0, 0, 1, 1)

capsule = new Capsule at (Range(-1.1, -0.9), Range(-0.1, 0.1), 0.83),
    with color (0.5, 0.5, 0.5, 1)

cylinder = new Cylinder at (Range(-1.1, -0.9), Range(-0.1, 0.1), 0.83),
    with color (0, 1, 1, 1)

# TABLE 2 (Front): Nuts and food items
square_nut = new SquareNut at (Range(0.9, 1.1), Range(-0.1, 0.1), 0.85)

round_nut = new RoundNut at (Range(0.9, 1.1), Range(-0.1, 0.1), 0.85)

milk = new Milk at (Range(0.9, 1.1), Range(-0.1, 0.1), 0.85)

cereal = new Cereal at (Range(0.9, 1.1), Range(-0.1, 0.1), 0.85)

# TABLE 3 (Right): Various objects
can = new Can at (Range(-0.1, 0.1), Range(0.9, 1.1), 0.85)

bread = new Bread at (Range(-0.1, 0.1), Range(0.9, 1.1), 0.85)

bottle = new Bottle at (Range(-0.1, 0.1), Range(0.9, 1.1), 0.85)

hammer = new Hammer at (Range(-0.1, 0.1), Range(0.9, 1.1), 0.85)

# TABLE 4 (Left): Intentionally left empty

# ROBOT
ego = new Panda at (0, 0, 0)