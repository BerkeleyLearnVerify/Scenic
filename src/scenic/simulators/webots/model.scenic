"""Generic Scenic world model for the Webots simulator."""

import math

from scenic.core.distributions import distributionFunction
from scenic.simulators.webots.actions import *

def _errorMsg():
    raise RuntimeError('scenario must be run from inside Webots')
simulator _errorMsg()

class WebotsObject:
    """Abstract class for Webots objects.

    The ``webotsName`` property must be set to the name of the Webots node to
    use for this object, which must already exist in the world loaded into
    Webots.

    Also defines the ``elevation`` property as a standard way to access the Y
    component of an object's position, since the Scenic built-in property
    ``position`` is only 2D. If ``elevation`` is set to :obj:`None`, it will be
    updated to the object's Y coordinate in Webots when the simulation starts.

    Properties:
        elevation (float or None; dynamic): default ``None`` (see above).
        requireVisible (bool): Default value ``False`` (overriding the default
            from `Object`).
        webotsName (str): 'DEF' name of the Webots node to use for this object.
        webotsObject: Is set at runtime to a handle to the Webots node for the
            object, for use with the `Supervisor API`_. Primarily for internal
            use.
        controller (str or None): name of the Webots controller to use for
            this object, if any (instead of a Scenic behavior).
        resetController (bool): Whether to restart the controller for each
            simulation (default ``True``).
        positionOffset (`Vector`): Offset to add when computing the object's
            position in Webots; for objects whose Webots ``translation`` field
            is not aligned with the center of the object.

    .. _Supervisor API: https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python
    """

    elevation[dynamic]: None
    requireVisible: False

    webotsName: None
    webotsType: None
    webotsObject: None

    controller: None
    resetController: True

    positionOffset: (0, 0)

class Ground(WebotsObject):
    """Special kind of object representing a (possibly irregular) ground surface.

    Implemented using an `ElevationGrid`_ node in Webots.

    Attributes:
        allowCollisions (bool): default value `False` (overriding default from `Object`).
        webotsName (str): default value 'Ground'

    .. _ElevationGrid: https://www.cyberbotics.com/doc/reference/elevationgrid
    """

    allowCollisions: True
    webotsName: 'Ground'
    positionOffset: (-self.width/2, -self.length/2)  # origin of ElevationGrid is at a corner

    gridSize: 20
    gridSizeX: self.gridSize
    gridSizeY: self.gridSize
    terrain: ()
    heights: Ground.heightsFromTerrain(self.terrain, self.gridSizeX, self.gridSizeY,
                                       self.width, self.length)

    @staticmethod
    @distributionFunction
    def heightsFromTerrain(terrain, gridSizeX, gridSizeY, width, length):
        for elem in terrain:
            if not isinstance(elem, Terrain):
                raise RuntimeError(f'Ground terrain element {elem} is not a Terrain')
        heights = []
        if gridSizeX < 2 or gridSizeY < 2:
            raise RuntimeError(f'invalid grid size {gridSizeX} x {gridSizeY} for Ground')
        dx, dy = width / (gridSizeX - 1), length / (gridSizeY - 1)
        y = -length / 2
        for i in range(gridSizeY):
            row = []
            x = -width / 2
            for j in range(gridSizeX):
                height = sum(elem.heightAt(x @ y) for elem in terrain)
                row.append(height)
                x += dx
            heights.append(tuple(row))
            y += dy
        return tuple(heights)

    def startDynamicSimulation(self):
        super().startDynamicSimulation()
        self.setGeometry()

    def setGeometry(self):
        # Set basic properties of grid
        shape = self.webotsObject.getField('children').getMFNode(0)
        grid = shape.getField('geometry').getSFNode()   # ElevationGrid node
        grid.getField('xDimension').setSFInt32(self.gridSizeX)
        grid.getField('zDimension').setSFInt32(self.gridSizeY)
        grid.getField('xSpacing').setSFFloat(self.width / (self.gridSizeX - 1))
        grid.getField('zSpacing').setSFFloat(self.length / (self.gridSizeY - 1))
        # Adjust length of height field as needed
        # (this will trigger Webots warnings, unfortunately; there seems to be no way to
        # update the length simultaneously with xDimension, etc.)
        heightField = grid.getField('height')
        count = heightField.getCount()
        size = self.gridSizeX * self.gridSizeY
        if count > size:
            for i in range(count - size):
                heightField.removeMF(-1)
        elif count < size:
            for i in range(size - count):
                heightField.insertMFFloat(-1, 0)
        # Set height values
        i = 0
        for row in self.heights:
            for height in row:
                heightField.setMFFloat(i, height)
                i += 1

class Terrain:
    """Abstract class for objects added together to make a `Ground`.

    This is not a `WebotsObject` since it doesn't actually correspond to a
    Webots node. Only the overall `Ground` has a node.
    """
    allowCollisions: True

    def heightAt(self, pt):
        offset = pt - self.position
        return self.heightAtOffset(offset)

    def heightAtOffset(self, offset):
        raise NotImplementedError('should be implemented by subclasses')

class Hill(Terrain):
    """`Terrain` shaped like a Gaussian.

    Attributes:
        height (float): height of the hill (default 1).
        spread (float): standard deviation as a fraction of the hill's size
            (default 3).
    """

    height: 1
    spread: 0.25

    def heightAtOffset(self, offset):
        dx, dy = offset
        if not (-self.hw < dx < self.hw and -self.hl < dy < self.hl):
            return 0
        sx, sy = dx / (self.width * self.spread), dy / (self.length * self.spread)
        nh = math.exp(-((sx * sx) + (sy * sy)) * 0.5)
        return self.height * nh
