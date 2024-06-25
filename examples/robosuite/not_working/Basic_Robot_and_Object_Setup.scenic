"""
Basic robot initialization with objects to manipulate.
"""

model scenic.simulators.robosuite.model

arena = new Table at (0, 0, 0)

# Create a Panda robot at origin
# panda = new PandaRobot at (0, 0, 0)

# Create robot positioned next to table, facing it
ego = new PandaRobot at (0, -0.8, 0),
    facing 90 deg  # Face positive Y direction (toward table)

# Create some objects to manipulate
red_cube = new Cube at (0.3, 0.0, 0.05),
    with color (0.8, 0.2, 0.2),
    with density 500

blue_ball = new Ball at (0.2, 0.2, 0.05),
    with color (0.2, 0.2, 0.8),
    with width 0.08

green_cylinder = new Cylinder at (-0.2, 0.1, 0.1),
    with color (0.2, 0.8, 0.2),
    with height 0.15

# Monitor to observe the scene
monitor SceneMonitor():
    for i in range(10):
        wait
        print(f"Step {i}:")
        print(f"  Cube at {red_cube.position}")
        print(f"  Ball at {blue_ball.position}, velocity: {blue_ball.velocity}")
        print(f"  Robot at {panda.position}")