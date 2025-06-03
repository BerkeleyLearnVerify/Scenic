# src/scenic/simulators/robosuite/model.scenic
"""Scenic world model for RoboSuite simulator."""

from .simulator import RobosuiteSimulator

simulator RobosuiteSimulator(render=True, real_time=True, speed=2.0)

# Base classes
class RoboSuiteObject(Object):
    """Base class for RoboSuite objects."""
    # Physics properties available to all objects
    density: 1000
    friction: (1.0, 0.005, 0.0001)
    solref: (0.02, 1.0)
    solimp: (0.9, 0.95, 0.001, 0.5, 2.0)