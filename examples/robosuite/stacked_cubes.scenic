model scenic.simulators.robosuite.model

# param camera_view = "sideview"

work_table = new Table at (0.6, 0, 0.8),
    with width 0.6,
    with length 1.2,
    with height 0.05

bottom_cube = new Box at (0.6, 0, 0.83),
    with color (0.2, 0.3, 0.8, 1),
    with width 0.06, with length 0.06, with height 0.06

top_cube = new Box at (0.6, 0, 0.89), 
    with color (0.8, 0.2, 0.2, 1),

bottle = new Bottle at (0.6, 0.3, 0.83)

ego = new UR5e at (0, 0, 0)