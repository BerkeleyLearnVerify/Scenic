model scenic.simulators.maniskill.model

# Create ground plane
ground = new Ground with position (0, 0, -0.920)

# Create three cubes stacked on top of each other
# Bottom cube (red)
cube1 = new ManiskillObject on ground,
    at (0, 0, 0.035),
    with name "cube1",
    with shape BoxShape(),
    with width 0.07,
    with length 0.07,
    with height 0.07,
    with color [0.9, 0.1, 0.1, 1.0]

# Middle cube (green)
cube2 = new ManiskillObject on cube1,
    with name "cube2",
    with shape BoxShape(),
    with width 0.07,
    with length 0.07,
    with height 0.07,
    with color [0.1, 0.9, 0.1, 1.0]

# Top cube (blue)
cube3 = new ManiskillObject on cube2,
    with name "cube3",
    with shape BoxShape(),
    with width 0.07,
    with length 0.07,
    with height 0.07,
    with color [0.1, 0.1, 0.9, 1.0]
