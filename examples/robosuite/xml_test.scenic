# examples/robosuite/xml_test.scenic
model scenic.simulators.robosuite.model

# Place table at ground level (z=0)
# table1 = new PositionableTable at (-0.5, 0, 0),
#     with width 0.8,
#     with length 0.8,
#     with height 0.8

table2 = new PositionableTable at (0.5, 0, 0),
    with width 0.6,
    with length 0.6,
    with height 0.6

# Cube on table - table height is 0.8, so cube should be at 0.8 + half cube height

cube1 = new Cube at (-0.5, 0, 0.825),
    with color (1, 0, 0)

cube2 = new Cube at (0.5, 0, 0.625),
    with color (0, 0, 1)

ego = cube1
terminate after 50 steps