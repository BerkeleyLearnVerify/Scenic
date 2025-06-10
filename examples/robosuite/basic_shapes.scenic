# examples/robosuite/basic_shapes.scenic
"""Basic shapes demonstration for RoboSuite integration."""

from scenic.simulators.robosuite.model import *

# Create a simple scene with basic shapes
cube = Cube at (0, 0, 0.05),
    with color (1, 0, 0)

ball = Ball at (0.1, 0, 0.05),
    with color (0, 1, 0)

cylinder = Cylinder at (-0.1, 0, 0),
    with color (0, 0, 1)

# Run for 100 steps
terminate after 100 steps