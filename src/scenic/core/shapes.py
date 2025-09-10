"""Module containing the Shape class and its subclasses, which represent shapes of Objects"""

from abc import ABC, abstractmethod

import numpy
import trimesh
from trimesh.transformations import (
    concatenate_matrices,
    quaternion_matrix,
    translation_matrix,
)

from scenic.core.type_support import toOrientation
from scenic.core.utils import cached_property, unifyMesh
from scenic.core.vectors import Orientation

###################################################################################################
# Abstract Classes and Utilities
###################################################################################################


class Shape(ABC):
    """An abstract base class for Scenic shapes.

    Represents a physical shape in Scenic. Does not encode position or orientation,
    which are handled by the `Region` class. Does contain dimension information, which
    is used as a default value by any `Object` with this shape and can be overwritten.

    If dimensions and scale are both specified the dimensions are first set by dimensions,
    and then scaled by scale.

    Args:
        dimensions: The raw (before scaling) dimensions of the shape.
        scale: Scales all the dimensions of the shape by a multiplicative factor.
    """

    def __init__(self, dimensions, scale):
        # Store values
        self.raw_dimensions = dimensions
        self.scale = scale
        self.dimensions = tuple(dim * self.scale for dim in self.raw_dimensions)
        self.width = self.dimensions[0]
        self.length = self.dimensions[1]
        self.height = self.dimensions[2]
        for dim, name in zip(self.dimensions, ("width", "length", "height")):
            if dim <= 0:
                raise ValueError(f"{name} of shape must be positive")

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
    """A Shape subclass defined by a `trimesh.base.Trimesh` object.

    The mesh passed must be a `trimesh.base.Trimesh` object that represents a well defined
    volume (i.e. the ``is_volume`` property must be true), meaning the mesh must be watertight,
    have consistent winding and have outward facing normals.

    Args:
        mesh: A mesh object.
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
        if not mesh.is_volume:
            raise ValueError(
                "A MeshShape cannot be defined with a mesh that does not have a well defined volume."
                " Consider using scenic.core.utils.repairMesh."
            )

        # Copy mesh and center vertices around origin
        self._mesh = mesh.copy()
        self._mesh.vertices -= self._mesh.bounding_box.center_mass

        # If rotation is provided, apply rotation
        if initial_rotation is not None:
            rotation = toOrientation(initial_rotation)
            rotation_matrix = quaternion_matrix(
                (rotation.w, rotation.x, rotation.y, rotation.z)
            )
            self._mesh.apply_transform(rotation_matrix)

        # If dimensions are not specified, infer them.
        if dimensions is None:
            dimensions = list(self._mesh.extents)

        # Scale mesh to unit size
        scale_vals = numpy.array(self._mesh.extents)
        scale_matrix = numpy.eye(4)
        scale_matrix[:3, :3] /= scale_vals
        self._mesh.apply_transform(scale_matrix)

        super().__init__(dimensions, scale)

    @classmethod
    def fromFile(cls, path, unify=True, **kwargs):
        """Load a mesh shape from a file, attempting to infer filetype and compression.

        For example: "foo.obj.bz2" is assumed to be a compressed .obj file.
        "foo.obj" is assumed to be an uncompressed .obj file. "foo" is an
        unknown filetype, so unless a filetype is provided an exception will be raised.

        Args:
            path (str): Path to the file to import.
            filetype (str): Filetype of file to be imported. This will be inferred if not provided.
                The filetype must be one compatible with `trimesh.load`.
            compressed (bool): Whether or not this file is compressed (with bz2). This will be inferred
                if not provided.
            binary (bool): Whether or not to open the file as a binary file.
            unify (bool): Whether or not to attempt to unify this mesh.
            kwargs: Additional arguments to the MeshShape initializer.
        """
        mesh = trimesh.load(path, force="mesh")
        if not mesh.is_volume:
            raise ValueError(
                "A MeshShape cannot be defined with a mesh that does not have a well defined volume."
                " Consider using scenic.core.utils.repairMesh."
            )
        if unify:
            mesh = unifyMesh(mesh, verbose=True)
        return cls(mesh, **kwargs)

    @property
    def mesh(self):
        return self._mesh

    @cached_property
    def isConvex(self):
        return self._mesh.is_convex

    def __getstate__(self):
        state = self.__dict__.copy()
        state["_mesh"] = self._mesh.copy()
        return state


class BoxShape(MeshShape):
    """A box shape with all dimensions 1 by default."""

    def __init__(self, dimensions=(1, 1, 1), scale=1, initial_rotation=None):
        super().__init__(
            trimesh.creation.box((1, 1, 1)), dimensions, scale, initial_rotation
        )


class CylinderShape(MeshShape):
    """A cylinder shape with all dimensions 1 by default."""

    def __init__(self, dimensions=(1, 1, 1), scale=1, initial_rotation=None, sections=24):
        super().__init__(
            trimesh.creation.cylinder(radius=0.5, height=1, sections=sections),
            dimensions,
            scale,
            initial_rotation,
        )
        self.sections = sections


class ConeShape(MeshShape):
    """A cone shape with all dimensions 1 by default."""

    def __init__(self, dimensions=(1, 1, 1), scale=1, initial_rotation=None):
        super().__init__(
            trimesh.creation.cone(radius=0.5, height=1),
            dimensions,
            scale,
            initial_rotation,
        )


class SpheroidShape(MeshShape):
    """A spheroid shape with all dimensions 1 by default."""

    def __init__(self, dimensions=(1, 1, 1), scale=1, initial_rotation=None):
        super().__init__(
            trimesh.creation.icosphere(radius=1), dimensions, scale, initial_rotation
        )
