
"""Common LGSVL interface."""

import math

import lgsvl

from scenic.core.vectors import Vector
from scenic.core.geometry import normalizeAngle

def lgsvlToScenicPosition(pos):
    """Convert LGSVL positions to Scenic positions."""
    return Vector(pos.x, pos.z)

def lgsvlToScenicElevation(pos):
    """Convert LGSVL positions to Scenic elevations."""
    return pos.y

def scenicToLGSVLPosition(pos, y=0):
    x, z = pos
    return lgsvl.Vector(x, y, z)

def lgsvlToScenicRotation(rot, tolerance2D=0):
    if abs(rot.x) > tolerance2D and abs(rot.x - 360) > tolerance2D:
        return None
    if abs(rot.z) > tolerance2D and abs(rot.z - 360) > tolerance2D:
        return None
    return normalizeAngle(-math.radians(rot.y))

def scenicToLGSVLRotation(heading):
    return lgsvl.Vector(0, -math.degrees(heading), 0)
