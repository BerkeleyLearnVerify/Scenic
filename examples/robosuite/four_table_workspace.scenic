# examples/robosuite/four_table_workspace.scenic
model scenic.simulators.robosuite.model

# SCENARIO CONFIGURATION
param use_environment = "Custom"
param camera_view = "sideview"

TABLE_DISTANCE = 1.0

# WORKSPACE: Four tables in cross formation
back_table = new Table at (-TABLE_DISTANCE, 0, 0.8)
front_table = new Table at (TABLE_DISTANCE, 0, 0.8)
right_table = new Table at (0, TABLE_DISTANCE, 0.8)
left_table = new Table at (0, -TABLE_DISTANCE, 0.8)

# TABLE 1 (Back): Primitive objects
ball = new CustomBall at (-1.0, 0, 0.82),
    with envObjectName "ball", with color (1, 0.5, 0, 1),
    with randomPlacement True, with tableIndex 0

box = new CustomBox at (-1.0, 0, 0.82),
    with envObjectName "box", with color (0, 0, 1, 1),
    with randomPlacement True, with tableIndex 0

capsule = new CustomCapsule at (-1.0, 0, 0.82),
    with envObjectName "capsule", with color (0.5, 0.5, 0.5, 1),
    with randomPlacement True, with tableIndex 0

cylinder = new CustomCylinder at (-1.0, 0, 0.82),
    with envObjectName "cylinder", with color (0, 1, 1, 1),
    with randomPlacement True, with tableIndex 0

# TABLE 2 (Front): cutsom objects 2
square_nut = new CustomSquareNut at (1.0, 0, 0.85),
    with envObjectName "square_nut", with randomPlacement True, with tableIndex 1

round_nut = new CustomRoundNut at (1.0, 0, 0.85),
    with envObjectName "round_nut", with randomPlacement True, with tableIndex 1

milk = new CustomMilk at (1.0, 0, 0.85),
    with envObjectName "milk", with randomPlacement True, with tableIndex 1

cereal = new CustomCereal at (1.0, 0, 0.85),
    with envObjectName "cereal", with randomPlacement True, with tableIndex 1

# TABLE 3 (Right): custom objects 1
can = new CustomCan at (0, 1.0, 0.85),
    with envObjectName "can", with randomPlacement True, with tableIndex 2

bread = new CustomBread at (0, 1.0, 0.85),
    with envObjectName "bread", with randomPlacement True, with tableIndex 2

bottle = new CustomBottle at (0, 1.0, 0.85),
    with envObjectName "bottle", with randomPlacement True, with tableIndex 2

hammer = new CustomHammer at (0, 1.0, 0.85),
    with envObjectName "hammer", with randomPlacement True, with tableIndex 2

# TABLE 4 (Left): Empty is left empty

# ROBOT
ego = new PandaRobot at (0, 0, 0)