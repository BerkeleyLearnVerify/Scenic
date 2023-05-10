""" Module containing the Shape class and its subclasses, which represent shapes of Objects"""

from abc import ABC, abstractmethod

import trimesh
from trimesh.transformations import (
    translation_matrix,
    quaternion_matrix,
    concatenate_matrices,
)
import numpy

from scenic.core.vectors import Orientation
from scenic.core.utils import cached_property, loadMesh

###################################################################################################
# Abstract Classes and Utilities
###################################################################################################


class Shape(ABC):
    """An abstract base class for Scenic shapes.

    Represents a physical shape in Scenic. Does not encode position or orientation,
    which are handled by the `Region` class. Does contain dimension information, which
    is used as a default value by any `Object` with this shape and can be overwritten.

    Args:
        dimensions: The raw (before scaling) dimensions of the shape. If dimensions
          and scale are both specified the dimensions are first set by dimensions, and then
          scaled by scale.
        scale: Scales all the dimensions of the shape by a multiplicative factor.
          If dimensions and scale are both specified the dimensions are first set by dimensions,
          and then scaled by scale.
    """

    def __init__(self, dimensions, scale):
        # Store values
        self.raw_dimensions = dimensions
        self.scale = scale
        self.dimensions = tuple(dim * self.scale for dim in self.raw_dimensions)
        self.width = self.dimensions[0]
        self.length = self.dimensions[1]
        self.height = self.dimensions[2]

    @cached_property
    def containsCenter(self):
        """Whether or not this object contains its central point"""
        pq = trimesh.proximity.ProximityQuery(self.mesh)
        region_distance = pq.signed_distance([(0, 0, 0)])[0]

        return region_distance > 0

    @property
    @abstractmethod
    def mesh(self):
        pass

    @property
    @abstractmethod
    def isConvex(self):
        pass


###################################################################################################
# 3D Shape Classes
###################################################################################################


class MeshShape(Shape):
    """A Shape subclass defined by a Trimesh object.

    Args:
        mesh: A trimesh.Trimesh mesh object.
        dimensions: The raw (before scaling) dimensions of the shape. If dimensions
          and scale are both specified the dimensions are first set by dimensions, and then
          scaled by scale.
        scale: Scales all the dimensions of the shape by a multiplicative factor.
          If dimensions and scale are both specified the dimensions are first set by dimensions,
          and then scaled by scale.
        initial_rotation: A 3-tuple containing the yaw, pitch, and roll respectively to apply when loading
          the mesh. Note the initial_rotation must be fixed.
    """

    def __init__(self, mesh, dimensions=None, scale=1, initial_rotation=None):
        # Ensure the mesh is watertight so volume is well defined
        if not mesh.is_watertight:
            raise ValueError(
                "A MeshShape cannot be defined with a mesh that does not have a well defined volume."
            )

        # Copy mesh and center vertices around origin
        self._mesh = mesh.copy()
        self._mesh.vertices -= self._mesh.bounding_box.center_mass
        self._isConvex = self._mesh.is_convex

        # If rotation is provided, apply rotation
        if initial_rotation is not None:
            rotation = Orientation.fromEuler(*initial_rotation)
            rotation_matrix = quaternion_matrix(
                (rotation.w, rotation.x, rotation.y, rotation.z)
            )
            self._mesh.apply_transform(rotation_matrix)

        # If dimensions are not specified, infer them.
        if dimensions is None:
            dimensions = list(self._mesh.extents)

        # Scale mesh to unit size
        scale_vals = self._mesh.extents / numpy.array([1, 1, 1])
        scale_matrix = numpy.eye(4)
        scale_matrix[:3, :3] /= scale_vals
        self._mesh.apply_transform(scale_matrix)

        # Report samplables
        super().__init__(dimensions, scale)

    @classmethod
    def fromFile(cls, path, filetype=None, compressed=None, **kwargs):
        """Load a shape from a file, attempting to infer filetype and compression.

        For example: "foo.obj.bz2" is assumed to be a compressed .obj file.
        "foo.obj" is assumed to be an uncompressed .obj file. "foo" is an
        unknown filetype, so unless a filetype is provided an exception will be raised.

        Args:
            path: Path to the file to import
            filetype: Filetype of file to be imported. This will be inferred if not provided
            compressed: Wheter or not this file is compressed (with bz2). This will be inferred
                if not provided
            kwargs: Additional arguments to the MeshShape initializer.
        """
        mesh = loadMesh(path, filetype, compressed)
        return cls(mesh, **kwargs)

    @property
    def mesh(self):
        return self._mesh

    @property
    def isConvex(self):
        return self._isConvex


class BoxShape(MeshShape):
    """A 3D box with all dimensions 1 by default."""

    def __init__(self, dimensions=(1, 1, 1), scale=1):
        super().__init__(trimesh.creation.box((1, 1, 1)), dimensions, scale)


class CylinderShape(MeshShape):
    def __init__(self, dimensions=(1, 1, 1), scale=1, sections=24):
        super().__init__(
            trimesh.creation.cylinder(radius=0.5, height=1, sections=sections),
            dimensions,
            scale,
        )
        self.sections = sections


class ConeShape(MeshShape):
    def __init__(self, dimensions=(1, 1, 1), scale=1):
        super().__init__(trimesh.creation.cone(radius=0.5, height=1), dimensions, scale)


class SpheroidShape(MeshShape):
    def __init__(self, dimensions=(1, 1, 1), scale=1):
        super().__init__(trimesh.creation.icosphere(radius=1), dimensions, scale)
