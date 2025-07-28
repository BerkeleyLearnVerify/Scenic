"""Scenic world model and simulator interface for RoboSuite.

This package provides:
- RobosuiteSimulator: Main simulator interface for Scenic
- World model definitions in model.scenic
- XML building utilities for custom objects
"""

from .simulator import RobosuiteSimulator

__all__ = ["RobosuiteSimulator"]