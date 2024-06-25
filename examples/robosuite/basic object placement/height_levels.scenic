# 
"""Test Suite 3: Random Height Levels with Sampling."""

model scenic.simulators.robosuite.model

class HeightCube(Object):
    width: 0.12
    length: 0.12
    height: 0.12

# Random cube positions and heights
ground_cube = new HeightCube at (Range(-0.4, -0.2), Range(-0.2, 0.2), Range(0.05, 0.1)), with color (0, 0, 0)
low_cube = new HeightCube at (Range(-0.2, 0.0), Range(-0.2, 0.2), Range(0.15, 0.25)), with color (1, 0, 0)
mid_cube = new HeightCube at (Range(0.0, 0.2), Range(-0.2, 0.2), Range(0.3, 0.5)), with color (0, 1, 0)
high_cube = new HeightCube at (Range(0.2, 0.4), Range(-0.2, 0.2), Range(0.5, 0.7)), with color (0, 0, 1)

# Ensure minimum separation between cubes
require (distance from ground_cube to low_cube) > 0.1
require (distance from low_cube to mid_cube) > 0.1
require (distance from mid_cube to high_cube) > 0.1

ego = mid_cube
record ego.position as egoPos