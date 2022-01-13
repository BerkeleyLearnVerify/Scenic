"""Various utilities for working with Webots scenarios."""

import math

import numpy as np

from scenic.core.geometry import normalizeAngle

def webotsToScenicPosition(pos):
    """Convert a Webots position to a Scenic position.

    Drops the Webots Y coordinate.
    """
    x, y, z = pos
    return (x, z)

def scenicToWebotsPosition(pos, y=0):
    """Convert a Scenic position to a Webots position."""
    x, z = pos
    return [x, y, z]

def webotsToScenicRotation(rot, tolerance2D=None):
    """Convert a Webots rotation vector to a Scenic heading.

    Assumes the object lies in the Webots X-Z plane, with a rotation axis
    close to the Y axis. If ``tolerance2D`` is given, returns ``None`` if the
    orientation of the object is not sufficiently close to being 2D.
    """
    axis = np.array(rot[:3])
    angle = rot[3]
    if tolerance2D is not None and np.linalg.norm(axis - (0, 1, 0)) > tolerance2D:
        return None
    return normalizeAngle(angle)

def scenicToWebotsRotation(heading):
    """Convert a Scenic heading to a Webots rotation vector."""
    return [0, 1, 0, heading]
