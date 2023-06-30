"""Generic Scenic world model for the Webots simulator.

This model provides a general type of object `WebotsObject` corresponding to a
node in the Webots scene tree, as well as a few more specialized objects.

Scenarios using this model cannot be launched directly from the command line
using the :option:`--simulate` option. Instead, Webots should be started first,
with a ``.wbt`` file that includes nodes for all the objects in the scenario
(see the `WebotsObject` documentation for how to specify which objects
correspond to which nodes). A supervisor node can then invoke Scenic to compile
the scenario and run dynamic simulations: see
:doc:`scenic.simulators.webots.simulator` for details.
"""

import math

import numpy
import trimesh

from scenic.core.object_types import Object2D
from scenic.core.shapes import MeshShape
from scenic.core.distributions import distributionFunction
from scenic.simulators.webots.actions import *

def _errorMsg():
    raise RuntimeError('scenario must be run from inside Webots')
simulator _errorMsg()

def is2DMode():
    from scenic.syntax.veneer import mode2D
    return mode2D

class WebotsObject:
    """Abstract class for Webots objects.

    There several ways to specify which Webots node this object corresponds to:

     * Set the ``webotsName`` property to the DEF name of the Webots node,
       which must already exist in the world loaded into Webots.

     * Set the ``webotsType`` property to a prefix like 'ROCK': the
       interface will then search for nodes called 'ROCK_0', 'ROCK_1', etc.
       Again the nodes must already exist in the world loaded into Webots.

     * Set the ``webotsAdhoc`` property to a dictionary of parameters. This will
       cause Scenic to dynamically create an Object in Webots, according to the
       parameters in the dictionary. **This is currently the only way to create
       objects in Webots that do not correspond to an existing node**. The parameters
       that can be contained in the dictionary are:

        * ``physics``: Whether or not physics should be enabled for this object.
          Default value is :python:`True`.

    Properties:
        elevation (float or None; dynamic): default ``None`` (see above).
        requireVisible (bool): Default value ``False`` (overriding the default
            from `Object`).
        webotsAdhoc (None | dict): None implies this object is not Adhoc. A dictionary
            implies this is an object that Scenic should create in Webots..
            If a dictionary, provides parameters for how to instantiate the adhoc object.
            See `scenic.simulators.webots.model` for more details.
        webotsName (str): 'DEF' name of the Webots node to use for this object.
        webotsType (str): If ``webotsName`` is not set, the first available
            node with 'DEF' name consisting of this string followed by '_0',
            '_1', etc. will be used for this object.
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
        rotationOffset (tuple[float, float, float]): Offset to add when computing the object's
            orientation in Webots; for objects whose front is not aligned with the
            Webots North axis.
        density (float): Density of this object in kg/m^3. The corresponding Webots object
            must have the ``density`` field.

    .. _Supervisor API: https://www.cyberbotics.com/doc/reference/supervisor?tab-language=python
    """
    elevation[dynamic]: None if is2DMode() else float(self.position.z)
    requireVisible: False

    webotsAdhoc: None

    webotsName: None
    webotsType: None
    webotsObject: None

    controller: None
    resetController: True

    positionOffset: (0, 0, 0)
    rotationOffset: (0, 0, 0)

    density: None

    @classmethod
    def _prepareSpecifiers(cls, specifiers):
        # Check specifiers for errors
        for spec in specifiers:
            # Raise error if elevation is specified outside of 2D mode
            if spec.name == "With" and tuple(spec.priorities.keys()) == ('elevation',):
                if not issubclass(cls, Object2D):
                    raise RuntimeError("Elevation being specified outside of 2D mode. "
                        "You should specify position's z component instead.")
        return specifiers

class Ground(WebotsObject):
    """Special kind of object representing a (possibly irregular) ground surface.

    Implemented using an `ElevationGrid`_ node in Webots.

    Attributes:
        allowCollisions (bool): default value `True` (overriding default from `Object`).
        webotsName (str): default value 'Ground'

    .. _ElevationGrid: https://www.cyberbotics.com/doc/reference/elevationgrid
    """

    allowCollisions: True
    webotsName: 'GROUND'
    positionOffset: (-self.width/2, -self.length/2, self.baseOffset[2])  # origin of ElevationGrid is at a corner

    baseOffset: Vector(0, 0, -(self.height)/2 + self.baseThickness)

    width: 10
    length: 10
    shape: Ground.shapeFromHeights(self.heights, self.width, self.length,
                self.gridSizeX, self.gridSizeY, self.baseThickness)

    gridSize: 20
    gridSizeX: self.gridSize
    gridSizeY: self.gridSize
    baseThickness: 0.1
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

    @staticmethod
    @distributionFunction
    def shapeFromHeights(heights, width, length, gridSizeX, gridSizeY, baseThickness):
        heights = [row[::-1] for row in heights[::-1]]

        triangles = []

        ## Vertices
        dx, dy = width / (gridSizeX - 1), length / (gridSizeY - 1)
        base_x, base_y = -width / 2, length / 2

        raw_vertices = numpy.asarray([[base_x+ix*dx, base_y-iy*dy, heights[iy][ix]]
            for iy in range(gridSizeY) for ix in range(gridSizeX)])
        heightmap_vertices = raw_vertices.reshape((gridSizeX, gridSizeY, 3))

        vertices =  numpy.vstack((raw_vertices, numpy.asarray(
                    [( width/2,-length/2,-baseThickness),
                     (-width/2,-length/2,-baseThickness),
                     (-width/2, length/2,-baseThickness),
                     ( width/2, length/2,-baseThickness)])))
        vertex_index_map = {tuple(vertices[i]):i for i in range(len(vertices))}

        # Create top surface
        for ix in range(gridSizeX-1):
            for iy in range(gridSizeY-1):
                # Calculate an interpolated middle point
                tl_x, tl_y, _ = heightmap_vertices[ix][iy]
                bl_x, bl_y, _ = heightmap_vertices[ix+1][iy+1]
                mean_height = (heightmap_vertices[ix][iy][2] + heightmap_vertices[ix+1][iy][2] +
                               heightmap_vertices[ix][iy+1][2] + heightmap_vertices[ix+1][iy+1][2])/4
                interpolated_point = ((tl_x + bl_x)/2, (tl_y + bl_y)/2, mean_height)

                triangles.extend([
                    (heightmap_vertices[ix][iy], interpolated_point, heightmap_vertices[ix][iy+1]), # Left
                    (heightmap_vertices[ix][iy], heightmap_vertices[ix+1][iy], interpolated_point), # Top
                    (heightmap_vertices[ix+1][iy], heightmap_vertices[ix+1][iy+1], interpolated_point), # Right
                    (heightmap_vertices[ix][iy+1], interpolated_point, heightmap_vertices[ix+1][iy+1]), # Bottom
                ])

        # Create side surfaces
        side_xy_vals = [(0, -width/2), (0, width/2), (1, -length/2), (1, length/2)]

        for side_vals in side_xy_vals:
            mid_side_vertices = [v for v in raw_vertices if v[side_vals[0]] == side_vals[1]]

            if side_vals[0] == 0:
                if side_vals[1] < 0:
                    side_vertices = [(-width/2, length/2, -baseThickness)] + mid_side_vertices + [(-width/2, -length/2, -baseThickness)] + [(-width/2, length/2, -baseThickness)]
                else:
                    side_vertices = [(width/2, length/2, -baseThickness)] + mid_side_vertices + [(width/2, -length/2, -baseThickness)] + [(width/2, length/2, -baseThickness)]
            else:
                if side_vals[1] < 0:
                    side_vertices = [(-width/2, -length/2, -baseThickness)] + mid_side_vertices + [(width/2, -length/2, -baseThickness)] + [(-width/2, -length/2, -baseThickness)]
                else:
                    side_vertices = [(-width/2, length/2, -baseThickness)] + mid_side_vertices + [(width/2, length/2, -baseThickness)] + [(-width/2, length/2, -baseThickness)]

            side_indices = [vertex_index_map[tuple(v)] for v in side_vertices]

            side_path = trimesh.path.Path3D(entities=[trimesh.path.entities.Line(side_indices)], vertices=vertices)

            flat_side_path, transform = side_path.to_planar(to_2D=None, normal=None, check=True)
            side_vertices_2d, side_faces = flat_side_path.triangulate()

            side_vertices_3d = trimesh.transformations.transform_points(
                numpy.hstack((side_vertices_2d,numpy.zeros([side_vertices_2d.shape[0],1], side_vertices_2d.dtype))), transform)

            side_triangles = [(side_vertices_3d[f1], side_vertices_3d[f3], side_vertices_3d[f2])  for f1, f2, f3 in side_faces]
            triangles += side_triangles

        # Create bottom surface
        triangles += [((-width/2,length/2,-baseThickness),(width/2,-length/2,-baseThickness),(-width/2,-length/2,-baseThickness)),
                      ((-width/2,length/2,-baseThickness),(width/2,-length/2,-baseThickness),(width/2,length/2,-baseThickness))]

        # Create and tune mesh
        heightmap_mesh = trimesh.Trimesh(**trimesh.triangles.to_kwargs(triangles))
        trimesh.repair.fix_winding(heightmap_mesh)
        heightmap_mesh.merge_vertices()

        assert heightmap_mesh.is_volume

        return MeshShape(heightmap_mesh)

    def startDynamicSimulation(self):
        super().startDynamicSimulation()
        self.setGeometry()

    def setGeometry(self):
        # Set basic properties of grid
        shape = self.webotsObject.getField('children').getMFNode(0)
        grid = shape.getField('geometry').getSFNode()   # ElevationGrid node
        grid.getField('xDimension').setSFInt32(self.gridSizeX)
        grid.getField('xSpacing').setSFFloat(self.width / (self.gridSizeX - 1))

        grid.getField('yDimension').setSFInt32(self.gridSizeY)
        grid.getField('ySpacing').setSFFloat(self.length / (self.gridSizeY - 1))

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
            for height in reversed(row):
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
    color: (0,0,0,0)

    def heightAtOffset(self, offset):
        dx, dy, _ = offset
        if not (-self.hw < dx < self.hw and -self.hl < dy < self.hl):
            return 0
        sx, sy = dx / (self.width * self.spread), dy / (self.length * self.spread)
        nh = math.exp(-((sx * sx) + (sy * sy)) * 0.5)
        return self.height * nh
