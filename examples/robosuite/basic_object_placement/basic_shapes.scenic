# test_suite_1_shapes.scenic
"""Test Suite 1: Different Shapes and Colors - FIXED."""

model scenic.simulators.robosuite.model

class TestCube(Object):
    width: 0.15
    length: 0.15
    height: 0.15
    # Remove shape property - use class name instead

class TestBall(Object):
    width: 0.15
    length: 0.15
    height: 0.15
    # Remove shape property - use class name instead

class TestCylinder(Object):
    width: 0.15
    length: 0.15
    height: 0.2
    # Remove shape property - use class name instead

# Different shapes in a line
red_cube = new TestCube at (-0.4, 0.0, 0.1), with color (1, 0, 0)
green_ball = new TestBall at (0.0, 0.0, 0.1), with color (0, 1, 0)  
blue_cylinder = new TestCylinder at (0.4, 0.0, 0.1), with color (0, 0, 1)

ego = green_ball
