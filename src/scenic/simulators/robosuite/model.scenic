# World model for the RoboSuite simulator.
# Defines objects and properties available in RoboSuite environments.

model scenic.simulators.robosuite.model

# Represents the Franka Emika Panda robot.
class Panda(Object):
    """A Scenic model for the Franka Emika Panda robot in RoboSuite."""
    # This property will be used to map this Scenic object to the
    # corresponding robot model name in RoboSuite.
    robosuiteName: 'Panda'
    width: 0.8
    length: 0.8

# A simple manipulable cube.
class ManipulableCube(Object):
    """A generic cube object for manipulation tasks."""
    width: 0.05
    length: 0.05
    height: 0.05

# TODO: Define a Workspace for RoboSuite environments.
# workspace = Workspace(...)