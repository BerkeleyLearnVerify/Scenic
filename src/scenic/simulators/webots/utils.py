"""Various utilities for working with Webots scenarios."""

import abc
import math

import numpy as np

from scenic.core.geometry import normalizeAngle

## Coordinate system transformations

class WebotsCoordinateSystem:
    """A Webots coordinate system into which Scenic positions can be converted.

    See the Webots documentation of `WorldInfo.coordinateSystem`_ for a discussion
    of the possible coordinate systems. Since Webots R2022a, the default coordinate
    axis convention is ENU (X-Y-Z=East-North-Up), which is the same as Scenic's.

    .. _WorldInfo.coordinateSystem: https://cyberbotics.com/doc/reference/worldinfo
    """
    def __init__(self, system='ENU'):
        axisMap = (system.find('E'), system.find('N'), system.find('U'))
        if len(system) != 3 or -1 in axisMap:
            raise RuntimeError('unknown kind of Webots coordinate system')
        self.axisMap = axisMap
        self.invAxisMap = (axisMap.index(0), axisMap.index(1), axisMap.index(2))
        self.upAxis = [0, 0, 0]
        self.upAxis[self.axisMap[2]] = 1
        self.leftHanded = (system[:2] in ('EU', 'NE', 'UN'))
        if self.leftHanded:
            self.mult = [1, -1, 1]
            self.invMult = [1, 1, -1]
        else:
            self.mult = self.invMult = [1, 1, 1]

    def positionToScenic(self, pos):
        """Convert a Webots position to a Scenic position."""
        return list(self.mult[i] * pos[self.axisMap[i]] for i in range(3))

    def positionFromScenic(self, pos, elevation=0):
        """Convert a Scenic position to a Webots position."""
        coords = (pos[0], pos[1], elevation)
        return list(self.invMult[i] * coords[self.invAxisMap[i]] for i in range(3))

    def rotationToScenic(self, rot, tolerance2D=None):
        """Convert a Webots rotation vector to a Scenic heading.

        Assumes the object lies in the Webots horizontal plane, with a rotation axis
        close to the up axis. If ``tolerance2D`` is given, returns ``None`` if the
        orientation of the object is not sufficiently close to being 2D.
        """
        axis = np.array(rot[:3])
        angle = rot[3]
        if tolerance2D is not None and np.linalg.norm(axis - self.upAxis) > tolerance2D:
            return None
        return normalizeAngle(angle)

    def rotationFromScenic(self, heading):
        """Convert a Scenic heading to a Webots rotation vector."""
        return self.upAxis + [heading]

ENU = WebotsCoordinateSystem('ENU') #: The ENU coordinate system (the Webots default).
NUE = WebotsCoordinateSystem('NUE') #: The NUE coordinate system.
EUN = WebotsCoordinateSystem('EUN') #: The EUN coordinate system.

# Old functions kept for backwards-compatibility

def webotsToScenicPosition(pos):
    """Convert a Webots position to a Scenic position.

    Drops the Webots Y coordinate.

    .. deprecated:: 2.1.0
        Use `WebotsCoordinateSystem.positionToScenic` instead, dropping the
        third coordinate.
    """
    x, y, z = pos
    return (x, z)

def scenicToWebotsPosition(pos, y=0, coordinateSystem='ENU'):
    """Convert a Scenic position to a Webots position.

    .. deprecated:: 2.1.0
        Use `WebotsCoordinateSystem.positionFromScenic` instead.
    """
    x, z = pos
    return [x, y, z]

def webotsToScenicRotation(rot, tolerance2D=None):
    """Convert a Webots rotation vector to a Scenic heading.

    Assumes the object lies in the Webots X-Z plane, with a rotation axis
    close to the Y axis. If ``tolerance2D`` is given, returns ``None`` if the
    orientation of the object is not sufficiently close to being 2D.

    .. deprecated:: 2.1.0
        Use `WebotsCoordinateSystem.rotationToScenic` instead.
    """
    axis = np.array(rot[:3])
    angle = rot[3]
    if tolerance2D is not None and np.linalg.norm(axis - (0, 1, 0)) > tolerance2D:
        return None
    return normalizeAngle(angle)

def scenicToWebotsRotation(heading):
    """Convert a Scenic heading to a Webots rotation vector.

    .. deprecated:: 2.1.0
        Use `WebotsCoordinateSystem.rotationFromScenic` instead.
    """
    return [0, 1, 0, heading]
