# test_suite_2_grid.scenic
"""Test Suite 2: Grid Formation."""

model scenic.simulators.robosuite.model

class GridCube(Object):
    width: 0.1
    length: 0.1
    height: 0.1

# 3x3 grid of cubes with different colors
cube_1_1 = new GridCube at (-0.2, -0.2, 0.05), with color (1, 0, 0)
cube_1_2 = new GridCube at (-0.2, 0.0, 0.05), with color (1, 0.5, 0)
cube_1_3 = new GridCube at (-0.2, 0.2, 0.05), with color (1, 1, 0)

cube_2_1 = new GridCube at (0.0, -0.2, 0.05), with color (0, 1, 0)
cube_2_2 = new GridCube at (0.0, 0.0, 0.05), with color (0, 1, 1)
cube_2_3 = new GridCube at (0.0, 0.2, 0.05), with color (0, 0, 1)

cube_3_1 = new GridCube at (0.2, -0.2, 0.05), with color (0.5, 0, 1)
cube_3_2 = new GridCube at (0.2, 0.0, 0.05), with color (1, 0, 1)
cube_3_3 = new GridCube at (0.2, 0.2, 0.05), with color (0.5, 0.5, 0.5)

ego = cube_2_2
