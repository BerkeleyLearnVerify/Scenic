
# test_suite_4_circle.scenic
"""Test Suite 4: Circular Formation."""

model scenic.simulators.robosuite.model

class CircleCube(Object):
    width: 0.08
    length: 0.08
    height: 0.08

# Arrange cubes in a circle
import math

radius = 0.3
cube_1 = new CircleCube at (radius * cos(0), radius * sin(0), 0.04), with color (1, 0, 0)
cube_2 = new CircleCube at (radius * cos(60 deg), radius * sin(60 deg), 0.04), with color (1, 0.5, 0)
cube_3 = new CircleCube at (radius * cos(120 deg), radius * sin(120 deg), 0.04), with color (1, 1, 0)
cube_4 = new CircleCube at (radius * cos(180 deg), radius * sin(180 deg), 0.04), with color (0, 1, 0)
cube_5 = new CircleCube at (radius * cos(240 deg), radius * sin(240 deg), 0.04), with color (0, 0, 1)
cube_6 = new CircleCube at (radius * cos(300 deg), radius * sin(300 deg), 0.04), with color (0.5, 0, 1)

# Center cube
center_cube = new CircleCube at (0.0, 0.0, 0.04), with color (1, 1, 1)

ego = center_cube
