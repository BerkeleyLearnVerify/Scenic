
"""Common LGSVL interface."""

import math

import lgsvl

from scenic.core.geometry import normalizeAngle

def lgsvlToScenicPosition(pos):
    """Convert LGSVL positions to Scenic positions."""
    return (pos.x, pos.z)

def lgsvlToScenicElevation(pos):
    """Convert LGSVL positions to Scenic elevations."""
    return pos.y

def scenicToLGSVLPosition(pos, y=0):
    x, z = pos
    return lgsvl.Vector(x, y, z)

def lgsvlToScenicRotation(rot, tolerance2D=0):
    if math.abs(rot.x) > tolerance2D or math.abs(rot.z) > tolerance2D:
        return None
    return normalizeAngle(math.radians(rot.y))

def scenicToLGSVLRotation(heading):
    return lgsvl.Vector(0, -math.degrees(heading), 0)
