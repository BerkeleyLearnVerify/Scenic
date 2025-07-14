# tests/simulators/robosuite/basic.scenic
"""Basic RoboSuite test scenario for headless CI testing.

This scenario creates simple objects without requiring external
model files or complex dependencies.
"""

# Import your python simulator class directly
from scenic.simulators.robosuite.simulator import RobosuiteSimulator

# Explicitly set the simulator for this scenario
simulator = RobosuiteSimulator

# Define basic object classes
class TestCube(Object):
    width: 0.05
    length: 0.05
    height: 0.05

class TestBall(Object):
    width: 0.04
    length: 0.04
    height: 0.04

class TestCylinder(Object):
    width: 0.06
    length: 0.06
    height: 0.08

# Create test objects with different shapes and colors
red_cube = new TestCube at (-0.2, 0.0, 0.1), with color (1, 0, 0)
green_ball = new TestBall at (0.0, 0.0, 0.1), with color (0, 1, 0)  
blue_cylinder = new TestCylinder at (0.2, 0.0, 0.1), with color (0, 0, 1)

# Ensure objects don't overlap
require (distance from red_cube to green_ball) > 0.1
require (distance from green_ball to blue_cylinder) > 0.1
require (distance from red_cube to blue_cylinder) > 0.3

ego = green_ball