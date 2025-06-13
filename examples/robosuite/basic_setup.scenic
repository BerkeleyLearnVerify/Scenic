# examples/robosuite/basic_setup.scenic
# Basic test scenario for the RoboSuite simulator interface.
#
# This scenario is intended to verify that the RoboSuite simulator
# can be instantiated by Scenic and that the world model is loaded correctly.
# It does not yet perform a dynamic simulation.

from scenic.simulators.robosuite.model import Panda, ManipulableCube

# Define the ego object as a Panda robot.
# In the future, the simulator will create this robot in RoboSuite.
ego = new Panda

# Define a cube to be manipulated.
new ManipulableCube at (0.5, 0.1, 0) relative to ego