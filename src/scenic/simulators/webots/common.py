
"""Common Webots interface."""

import math

import numpy as np

from scenic.core.geometry import normalizeAngle

def webotsToScenicPosition(pos):
    """Convert Webots positions to Scenic positions."""
    x, y, z = pos
    return (x, -z)

def scenicToWebotsPosition(pos, y=0):
    x, z = pos
    return [x, y, -z]

def webotsToScenicRotation(rot, tolerance2D=0):
    axis = np.array(rot[:3])
    angle = rot[3]
    if np.linalg.norm(axis - (0, 1, 0)) > tolerance2D:
        return None
    return normalizeAngle(angle + math.pi)

def scenicToWebotsRotation(heading):
    return [0, 1, 0, heading - math.pi]
