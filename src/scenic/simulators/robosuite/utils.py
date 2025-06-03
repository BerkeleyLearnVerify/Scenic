# src/scenic/simulators/robosuite/utils.py
"""Utility functions for RoboSuite-Scenic integration."""

from typing import List


def scenic_to_rgba(color, alpha=1.0) -> List[float]:
    """Convert Scenic color to RGBA format."""
    if hasattr(color, '__iter__') and len(color) >= 3:
        rgba = list(color[:4]) if len(color) >= 4 else list(color[:3]) + [alpha]
        return [float(x) for x in rgba]
    return [0.5, 0.5, 0.5, float(alpha)]