# test_suite_3_heights.scenic
"""Test Suite 3: Different Height Levels."""

class HeightCube(Object):
    width: 0.12
    length: 0.12
    height: 0.12

# Cubes at different height levels
ground_cube = new HeightCube at (-0.3, 0.0, 0.06), with color (0.5, 0.3, 0.1)  # Brown - ground level
low_cube = new HeightCube at (-0.1, 0.0, 0.2), with color (1, 0, 0)            # Red - low
mid_cube = new HeightCube at (0.1, 0.0, 0.4), with color (0, 1, 0)             # Green - medium  
high_cube = new HeightCube at (0.3, 0.0, 0.6), with color (0, 0, 1)            # Blue - high

ego = mid_cube
