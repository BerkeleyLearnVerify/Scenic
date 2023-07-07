"""Various utilities for working with Webots scenarios."""

import abc
import math
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation as R

from scenic.core.geometry import normalizeAngle
from scenic.core.vectors import Orientation

## Coordinate system transformations


class WebotsCoordinateSystem:
    """A Webots coordinate system into which Scenic positions can be converted.

    See the Webots documentation of `WorldInfo.coordinateSystem`_ for a discussion
    of the possible coordinate systems. Since Webots R2022a, the default coordinate
    axis convention is ENU (X-Y-Z=East-North-Up), which is the same as Scenic's.

    .. _WorldInfo.coordinateSystem: https://cyberbotics.com/doc/reference/worldinfo
    """

    def __init__(self, system="ENU"):
        self.system = system
        axisMap = (system.find("E"), system.find("N"), system.find("U"))
        if len(system) != 3 or -1 in axisMap:
            raise RuntimeError("unknown kind of Webots coordinate system")
        self.axisMap = axisMap
        self.invAxisMap = (axisMap.index(0), axisMap.index(1), axisMap.index(2))
        self.upAxis = [0, 0, 0]
        self.upAxis[self.axisMap[2]] = 1
        self.leftHanded = system[:2] in ("EU", "NE", "UN")
        if self.leftHanded:
            self.mult = [1, -1, 1]
            self.invMult = [1, 1, -1]
        else:
            self.mult = self.invMult = [1, 1, 1]

    def positionToScenic(self, pos):
        """Convert a Webots position to a Scenic position."""
        return list(self.mult[i] * pos[self.axisMap[i]] for i in range(3))

    def positionFromScenic(self, pos):
        """Convert a Scenic position to a Webots position."""
        return list(self.invMult[i] * pos[self.invAxisMap[i]] for i in range(3))

    def orientationFromScenic(
        self, orientation: Orientation, offset: Orientation
    ) -> List[float]:
        # TODO(shun): Support other coordinate systems
        if self.system != "ENU":
            raise ValueError("Coordinate systems other than ENU is not fully supported")

        target = orientation * offset
        r = target.getRotation()
        rotvec = r.as_rotvec()
        if rotvec.tolist() == [0, 0, 0]:
            webotsRotation = [1.0, 0.0, 0.0, 0.0]
        else:
            norm = np.linalg.norm(rotvec)
            rotvec = rotvec / norm
            webotsRotation = rotvec.tolist() + [norm]
        return webotsRotation

    def orientationToScenic(
        self, webotsOrientation: List[float], offset: Orientation
    ) -> Orientation:
        # TODO(shun): Support other coordinate systems
        if self.system != "ENU":
            raise ValueError("Coordinate systems other than ENU is not fully supported")

        # webotsOrientation is a list of length 4 whose first three values are the normalized rotation axis and
        # the fourth value is the rotation angle in radian
        angle = webotsOrientation[3]
        rotvec = np.array(webotsOrientation[0:3]) * angle
        target = Orientation(R.from_rotvec(rotvec.tolist()))
        orientation = target * offset.inverse
        return orientation


ENU = WebotsCoordinateSystem("ENU")  #: The ENU coordinate system (the Webots default).
NUE = WebotsCoordinateSystem("NUE")  #: The NUE coordinate system.
EUN = WebotsCoordinateSystem("EUN")  #: The EUN coordinate system.
