"""Objects representing regions in space.

Manipulations of polygons and line segments are done using the
`shapely <https://github.com/shapely/shapely>`_ package.

Manipulations of meshes is done using the
`trimesh <https://trimsh.org/>`_ package.
"""

from abc import ABC, abstractmethod
import itertools
import math
import random
import warnings

import numpy
import scipy
import shapely
import shapely.geometry
from shapely.geometry import MultiPolygon
import shapely.ops
import trimesh
from trimesh.transformations import (
    concatenate_matrices,
    quaternion_matrix,
    translation_matrix,
)

warnings.filterwarnings(
    "ignore", module="trimesh"
)  # temporarily suppress annoying warnings

from scenic.core.distributions import (
    RejectionException,
    Samplable,
    distributionFunction,
    distributionMethod,
    needsLazyEvaluation,
    needsSampling,
    toDistribution,
)
from scenic.core.geometry import (
    averageVectors,
    cos,
    findMinMax,
    headingOfSegment,
    hypot,
    makeShapelyPoint,
    plotPolygon,
    pointIsInCone,
    polygonUnion,
    sin,
    triangulatePolygon,
)
from scenic.core.lazy_eval import isLazy, valueInContext
from scenic.core.type_support import toOrientation, toScalar, toVector
from scenic.core.utils import cached, cached_method, cached_property, loadMesh, unifyMesh
from scenic.core.vectors import (
    Orientation,
    OrientedVector,
    Vector,
    VectorDistribution,
    VectorField,
)

###################################################################################################
# Abstract Classes and Utilities
###################################################################################################


class Region(Samplable, ABC):
    """An abstract base class for Scenic Regions"""

    def __init__(self, name, *dependencies, orientation=None):
        super().__init__(dependencies)
        self.name = name
        self.orientation = orientation

    ## Abstract Methods ##
    @abstractmethod
    def uniformPointInner(self):
        """Do the actual random sampling. Implemented by subclasses."""
        pass

    @abstractmethod
    def containsPoint(self, point) -> bool:
        """Check if the `Region` contains a point. Implemented by subclasses."""
        pass

    @abstractmethod
    def containsObject(self, obj) -> bool:
        """Check if the `Region` contains an :obj:`~scenic.core.object_types.Object`"""
        pass

    @abstractmethod
    def containsRegionInner(self, reg, tolerance) -> bool:
        """Check if the `Region` contains a `Region`"""
        pass

    @abstractmethod
    def distanceTo(self, point) -> float:
        """Distance to this region from a given point."""
        pass

    @abstractmethod
    def projectVector(self, point, onDirection):
        """Returns point projected onto this region along onDirection."""
        pass

    @property
    @abstractmethod
    def AABB(self):
        """Axis-aligned bounding box for this `Region`."""
        pass

    ## Overridable Methods ##
    # The following methods can be overriden to get better performance or if the region
    # has dependencies (in the case of sampleGiven).

    @property
    def dimensionality(self):
        return None

    @property
    def size(self):
        return None

    def intersects(self, other, triedReversed=False) -> bool:
        """intersects(other)

        Check if this `Region` intersects another.
        """
        if triedReversed:
            # Last ditch attempt. Try computing intersection and see if we get a
            # fixed result back.
            intersection = self.intersect(other)

            if isinstance(intersection, IntersectionRegion):
                raise NotImplementedError(
                    f"Cannot check intersection of {type(self).__name__} and {type(other).__name__}"
                )
            elif isinstance(intersection, EmptyRegion):
                return False
            else:
                return True
        else:
            return other.intersects(self, triedReversed=True)

    def intersect(self, other, triedReversed=False) -> "Region":
        """Get a `Region` representing the intersection of this one with another.

        If both regions have a :term:`preferred orientation`, the one of ``self``
        is inherited by the intersection.
        """
        if triedReversed:
            orientation = orientationFor(self, other, triedReversed)
            return IntersectionRegion(self, other, orientation=orientation)
        else:
            return other.intersect(self, triedReversed=True)

    def union(self, other, triedReversed=False) -> "Region":
        """Get a `Region` representing the union of this one with another.

        Not supported by all region types.
        """
        if triedReversed:
            return UnionRegion(self, other)
        else:
            return other.union(self, triedReversed=True)

    def difference(self, other) -> "Region":
        """Get a `Region` representing the difference of this one and another.

        Not supported by all region types.
        """
        if isinstance(other, EmptyRegion):
            return self
        elif isinstance(other, AllRegion):
            return nowhere
        return DifferenceRegion(self, other)

    def sampleGiven(self, value):
        return self

    ## Generic Methods (not to be overriden by subclasses) ##
    @cached_method
    def containsRegion(self, reg, tolerance=0):
        # Default behavior for AllRegion and EmptyRegion
        if type(self) is AllRegion or type(reg) is EmptyRegion:
            return True

        if type(self) is EmptyRegion or type(reg) is AllRegion:
            return False

        # Fast checks based off of dimensionality and size
        if self.dimensionality is not None and reg.dimensionality is not None:
            # A lower dimensional region cannot contain a higher dimensional region.
            if self.dimensionality < reg.dimensionality:
                return False

            if self.size is not None and reg.size is not None:
                # A smaller region cannot contain a larger region of the
                # same dimensionality.
                if self.dimensionality == reg.dimensionality and self.size < reg.size:
                    return False

        return self.containsRegionInner(reg, tolerance)

    @staticmethod
    def uniformPointIn(region):
        """Get a uniform `Distribution` over points in a `Region`."""
        return PointInRegionDistribution(region)

    def __contains__(self, thing) -> bool:
        """Check if this `Region` contains an object or vector."""
        from scenic.core.object_types import Object

        if isinstance(thing, Object):
            return self.containsObject(thing)
        vec = toVector(thing, '"X in Y" with X not an Object or a vector')
        return self.containsPoint(vec)

    def orient(self, vec):
        """Orient the given vector along the region's orientation, if any."""
        if self.orientation is None:
            return vec
        else:
            return OrientedVector(vec.x, vec.y, vec.z, self.orientation[vec])

    def __str__(self):
        s = f"<{type(self).__name__}"
        if self.name:
            s += f" {self.name}"
        return s + ">"

    def __repr__(self):
        s = f"<{type(self).__name__}"
        if self.name:
            s += f" {self.name}"
        return s + f" at {hex(id(self))}>"


class PointInRegionDistribution(VectorDistribution):
    """Uniform distribution over points in a Region"""

    def __init__(self, region):
        super().__init__(region)
        self.region = region

    def sampleGiven(self, value):
        return value[self.region].uniformPointInner()

    @property
    def heading(self):
        if self.region.orientation is not None:
            return self.region.orientation[self]
        else:
            return 0

    @property
    def z(self) -> float:
        # Simplify expression forest in some cases where z is known.
        reg = self.region
        if isinstance(reg, (GridRegion, PolylineRegion)):
            return 0.0
        if isinstance(reg, PolygonalRegion):
            return reg.z
        return super().z

    def __repr__(self):
        return f"PointIn({self.region!r})"


###################################################################################################
# Utility Regions and Functions
###################################################################################################


class AllRegion(Region):
    """Region consisting of all space."""

    def intersect(self, other, triedReversed=False):
        return other

    def intersects(self, other, triedReversed=False):
        return not isinstance(other, EmptyRegion)

    def union(self, other, triedReversed=False):
        return self

    def uniformPointInner(self):
        raise RuntimeError(f"Attempted to sample from everywhere (AllRegion)")

    def containsPoint(self, point):
        return True

    def containsObject(self, obj):
        return True

    def containsRegionInner(self, reg, tolerance):
        assert False

    def distanceTo(self, point):
        return 0

    def projectVector(self, point, onDirection):
        return point

    @property
    def AABB(self):
        raise TypeError("AllRegion does not have a well defined AABB")

    @property
    def dimensionality(self):
        return float("inf")

    @property
    def size(self):
        return float("inf")

    def __eq__(self, other):
        return type(other) is AllRegion

    def __hash__(self):
        return hash(AllRegion)


class EmptyRegion(Region):
    """Region containing no points."""

    def intersect(self, other, triedReversed=False):
        return self

    def intersects(self, other, triedReversed=False):
        return False

    def difference(self, other):
        return self

    def union(self, other, triedReversed=False):
        return other

    def uniformPointInner(self):
        raise RejectionException(f"sampling empty Region")

    def containsPoint(self, point):
        return False

    def containsObject(self, obj):
        return False

    def containsRegionInner(self, reg, tolerance):
        assert False

    def distanceTo(self, point):
        return float("inf")

    def projectVector(self, point, onDirection):
        raise RejectionException("Projecting vector onto empty Region")

    @property
    def AABB(self):
        raise TypeError("EmptyRegion does not have a well defined AABB")

    @property
    def dimensionality(self):
        return 0

    @property
    def size(self):
        return 0

    def show(self, plt, style=None, **kwargs):
        pass

    def __eq__(self, other):
        return type(other) is EmptyRegion

    def __hash__(self):
        return hash(EmptyRegion)


#: A `Region` containing all points.
#:
#: Points may not be sampled from this region, as no uniform distribution over it exists.
everywhere = AllRegion("everywhere")

#: A `Region` containing no points.
#:
#: Attempting to sample from this region causes the sample to be rejected.
nowhere = EmptyRegion("nowhere")


class IntersectionRegion(Region):
    def __init__(self, *regions, orientation=None, sampler=None, name=None):
        self.regions = tuple(regions)
        if len(self.regions) < 2:
            raise ValueError("tried to take intersection of fewer than 2 regions")
        super().__init__(name, *self.regions, orientation=orientation)
        self.sampler = sampler

    def sampleGiven(self, value):
        regs = [value[reg] for reg in self.regions]
        # Now that regions have been sampled, attempt intersection again in the hopes
        # there is a specialized sampler to handle it (unless we already have one)
        if not self.sampler:
            failed = False
            intersection = regs[0]
            for region in regs[1:]:
                intersection = intersection.intersect(region)
                if isinstance(intersection, IntersectionRegion):
                    failed = True
                    break
            if not failed:
                intersection.orientation = value[self.orientation]
                return intersection
        return IntersectionRegion(
            *regs,
            orientation=value[self.orientation],
            sampler=self.sampler,
            name=self.name,
        )

    def evaluateInner(self, context):
        regs = (valueInContext(reg, context) for reg in self.regions)
        orientation = valueInContext(self.orientation, context)
        return IntersectionRegion(
            *regs, orientation=orientation, sampler=self.sampler, name=self.name
        )

    def containsPoint(self, point):
        return all(region.containsPoint(point) for region in self.footprint.regions)

    def containsObject(self, obj):
        return all(region.containsObject(obj) for region in self.footprint.regions)

    def containsRegionInner(self, reg, tolerance):
        return all(region.containsRegion(reg, tolerance) for region in self.regions)

    def distanceTo(self, point):
        raise NotImplementedError

    def projectVector(self, point, onDirection):
        raise NotImplementedError(
            f'{type(self).__name__} does not yet support projection using "on"'
        )

    @property
    def AABB(self):
        raise NotImplementedError

    @cached_property
    def footprint(self):
        return convertToFootprint(self)

    def uniformPointInner(self):
        sampler = self.sampler
        if not sampler:
            sampler = self.genericSampler
        return self.orient(sampler(self))

    @staticmethod
    def genericSampler(intersection):
        regs = intersection.regions

        # Get a candidate point from each region
        points = []

        num_regs_undefined = 0

        for reg in regs:
            try:
                points.append(reg.uniformPointInner())
            except UndefinedSamplingException:
                num_regs_undefined += 1
                pass

        if num_regs_undefined == len(regs):
            # All regions do not support sampling, so the
            # intersection doesn't either.
            raise UndefinedSamplingException(
                f"All regions in {regs}"
                " do not support sampling, so the intersection doesn't either."
            )

        # Check each point for containment each region.
        for point in points:
            if all(region.containsPoint(point) for region in regs):
                return point

        raise RejectionException(f"sampling intersection of Regions {regs}")

    def __repr__(self):
        return f"IntersectionRegion({self.regions!r})"


class UnionRegion(Region):
    def __init__(self, *regions, orientation=None, sampler=None, name=None):
        self.regions = tuple(regions)
        if len(self.regions) < 2:
            raise ValueError("tried to take union of fewer than 2 regions")
        super().__init__(name, *self.regions, orientation=orientation)
        self.sampler = sampler

    def sampleGiven(self, value):
        regs = [value[reg] for reg in self.regions]
        # Now that regions have been sampled, attempt union again in the hopes
        # there is a specialized sampler to handle it (unless we already have one)
        if not self.sampler:
            failed = False
            union = regs[0]
            for region in regs[1:]:
                union = union.union(region)
                if isinstance(union, UnionRegion):
                    failed = True
                    break
            if not failed:
                union.orientation = value[self.orientation]
                return union

        return UnionRegion(
            *regs,
            orientation=value[self.orientation],
            sampler=self.sampler,
            name=self.name,
        )

    def evaluateInner(self, context):
        regs = (valueInContext(reg, context) for reg in self.regions)
        orientation = valueInContext(self.orientation, context)
        return UnionRegion(
            *regs, orientation=orientation, sampler=self.sampler, name=self.name
        )

    def containsPoint(self, point):
        return any(region.containsPoint(point) for region in self.footprint.regions)

    def containsObject(self, obj):
        raise NotImplementedError

    def containsRegionInner(self, reg, tolerance):
        raise NotImplementedError

    def distanceTo(self, point):
        raise NotImplementedError

    def projectVector(self, point, onDirection):
        raise NotImplementedError(
            f'{type(self).__name__} does not yet support projection using "on"'
        )

    @property
    def AABB(self):
        raise NotImplementedError

    @cached_property
    def footprint(self):
        return convertToFootprint(self)

    def uniformPointInner(self):
        sampler = self.sampler
        if not sampler:
            sampler = self.genericSampler
        return self.orient(sampler(self))

    @staticmethod
    def genericSampler(union):
        regs = intersection.regions

        # Check that all regions have well defined dimensionality
        if any(reg.dimensionality is None for reg in regs):
            raise UndefinedSamplingException(
                f"cannot sample union of Regions {regs} with " "undefined dimensionality"
            )

        # Filter out all regions with 0 probability
        max_dim = max(reg.dimensionality for reg in regs)
        large_regs = tuple(reg for reg in regs if reg.dimensionality == max_dim)

        # Check that all large regions have well defined size
        if any(reg.size is None or reg.size == float("inf") for reg in large_regs):
            raise UndefinedSamplingException(
                f"cannot sample union of Regions {regs} with " "ill-defined size"
            )

        # Pick a sample, weighted by region size
        reg_sizes = tuple(reg.size for reg in large_regs)
        target_reg = random.choices(large_regs, weights=reg_sizes)[0]
        point = target_reg.uniformPointInner()

        # Potentially reject based on containment of the sample
        containment_count = sum(int(reg.containsPoint(point)) for reg in regs)

        if random.random() < 1 - 1 / containment_count:
            raise RejectionException("rejected sample from UnionRegion")

        return point

    def __repr__(self):
        return f"UnionRegion({self.regions!r})"


class DifferenceRegion(Region):
    def __init__(self, regionA, regionB, sampler=None, name=None):
        self.regionA, self.regionB = regionA, regionB
        super().__init__(name, regionA, regionB, orientation=regionA.orientation)
        self.sampler = sampler

    def sampleGiven(self, value):
        regionA, regionB = value[self.regionA], value[self.regionB]
        # Now that regions have been sampled, attempt difference again in the hopes
        # there is a specialized sampler to handle it (unless we already have one)
        if not self.sampler:
            diff = regionA.difference(regionB)
            if not isinstance(diff, DifferenceRegion):
                diff.orientation = value[self.orientation]
                return diff
        return DifferenceRegion(regionA, regionB, sampler=self.sampler, name=self.name)

    def evaluateInner(self, context):
        regionA = valueInContext(self.regionA, context)
        regionB = valueInContext(self.regionB, context)
        orientation = valueInContext(self.orientation, context)
        return DifferenceRegion(
            regionA,
            regionB,
            orientation=orientation,
            sampler=self.sampler,
            name=self.name,
        )

    def containsPoint(self, point):
        return self.footprint.regionA.containsPoint(
            point
        ) and not self.footprint.regionB.containsPoint(point)

    def containsObject(self, obj):
        return self.footprint.regionA.containsObject(
            obj
        ) and not self.footprint.regionB.intersects(obj.occupiedSpace)

    def containsRegionInner(self, reg, tolerance):
        raise NotImplementedError

    def distanceTo(self, point):
        raise NotImplementedError

    def projectVector(self, point, onDirection):
        raise NotImplementedError(
            f'{type(self).__name__} does not yet support projection using "on"'
        )

    @property
    def AABB(self):
        raise NotImplementedError

    @cached_property
    def footprint(self):
        return convertToFootprint(self)

    def uniformPointInner(self):
        sampler = self.sampler
        if not sampler:
            sampler = self.genericSampler
        return self.orient(sampler(self))

    @staticmethod
    def genericSampler(difference):
        regionA, regionB = difference.regionA, difference.regionB
        point = regionA.uniformPointInner()
        if regionB.containsPoint(point):
            raise RejectionException(
                f"sampling difference of Regions {regionA} and {regionB}"
            )
        return point

    def __repr__(self):
        return f"DifferenceRegion({self.regionA!r}, {self.regionB!r})"


def toPolygon(thing):
    if needsSampling(thing):
        return None
    if hasattr(thing, "polygon"):
        poly = thing.polygon
    elif hasattr(thing, "polygons"):
        poly = thing.polygons
    elif hasattr(thing, "lineString"):
        poly = thing.lineString
    else:
        return None

    return poly


def regionFromShapelyObject(obj, orientation=None):
    """Build a 'Region' from Shapely geometry."""
    assert obj.is_valid, obj
    if obj.is_empty:
        return nowhere
    elif isinstance(obj, (shapely.geometry.Polygon, shapely.geometry.MultiPolygon)):
        return PolygonalRegion(polygon=obj, orientation=orientation)
    elif isinstance(obj, (shapely.geometry.LineString, shapely.geometry.MultiLineString)):
        return PolylineRegion(polyline=obj, orientation=orientation)
    elif isinstance(obj, shapely.geometry.MultiPoint):
        points = [pt.coords[0] for pt in obj.geoms]
        return PointSetRegion("PointSet", points, orientation=orientation)
    elif isinstance(obj, shapely.geometry.Point):
        return PointSetRegion("PointSet", obj.coords, orientation=orientation)
    else:
        raise TypeError(f"unhandled type of Shapely geometry: {obj}")


def orientationFor(first, second, reversed):
    o1 = first.orientation
    o2 = second.orientation
    if reversed:
        o1, o2 = o2, o1
    if not o1:
        o1 = o2
    return o1


class UndefinedSamplingException(Exception):
    pass


###################################################################################################
# 3D Regions
###################################################################################################


class SurfaceCollisionTrimesh(trimesh.Trimesh):
    """A Trimesh object that always returns non-convex.

    Used so that fcl doesn't find collision without an actual surface
    intersection.
    """

    @property
    def is_convex(self):
        return False


class MeshRegion(Region):
    """Region given by a scaled, positioned, and rotated mesh.

    This is an abstract class and cannot be instantiated directly. Instead a subclass should be used, like
    `MeshVolumeRegion` or `MeshSurfaceRegion`.

    The mesh is first placed so the origin is at the center of the bounding box (unless ``centerMesh`` is ``False``).
    The mesh is scaled to ``dimensions``, translated so the center of the bounding box of the mesh is at ``positon``,
    and then rotated to ``rotation``.

    Meshes are centered by default (since ``centerMesh`` is true by default). If you disable this operation, do note
    that scaling and rotation transformations may not behave as expected, since they are performed around the origin.

    Args:
        mesh: The base mesh for this MeshRegion.
        name: An optional name to help with debugging.
        dimensions: An optional 3-tuple, with the values representing width, length, height respectively.
          The mesh will be scaled such that the bounding box for the mesh has these dimensions.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for internal computations.
        centerMesh: Whether or not to center the mesh after copying and before transformations.
        onDirection: The direction to use if an object being placed on this region doesn't specify one.
        engine: Which engine to use for mesh operations. Either "blender" or "scad".
        additionalDeps: Any additional sampling dependencies this region relies on.
    """

    def __init__(
        self,
        mesh,
        dimensions=None,
        position=None,
        rotation=None,
        orientation=None,
        tolerance=1e-6,
        centerMesh=True,
        onDirection=None,
        engine="scad",
        name=None,
        additionalDeps=[],
    ):
        # Copy parameters
        self._mesh = mesh
        self.dimensions = None if dimensions is None else toVector(dimensions)
        self.position = None if position is None else toVector(position)
        self.rotation = None if rotation is None else toOrientation(rotation)
        self.orientation = None if orientation is None else toDistribution(orientation)
        self.tolerance = tolerance
        self.centerMesh = centerMesh
        self.onDirection = onDirection
        self.engine = engine

        # Initialize superclass with samplables
        super().__init__(
            name,
            self._mesh,
            self.dimensions,
            self.position,
            self.rotation,
            *additionalDeps,
            orientation=orientation,
        )

        # If our region isn't fixed yet, then compute other values later
        if isLazy(self):
            return

        # Convert extract mesh
        if isinstance(mesh, trimesh.primitives._Primitive):
            self._mesh = mesh.to_mesh()
        elif isinstance(mesh, trimesh.base.Trimesh):
            self._mesh = mesh.copy()
        else:
            raise TypeError(
                f"Got unexpected mesh parameter of type {type(mesh).__name__}"
            )

        # Center mesh unless disabled
        if centerMesh:
            self.mesh.vertices -= self.mesh.bounding_box.center_mass

        # If dimensions are provided, scale mesh to those dimension
        if self.dimensions is not None:
            scale = self.mesh.extents / numpy.array(self.dimensions)

            scale_matrix = numpy.eye(4)
            scale_matrix[:3, :3] /= scale

            self.mesh.apply_transform(scale_matrix)

        # If rotation is provided, apply rotation
        if self.rotation is not None:
            rotation_matrix = quaternion_matrix(
                (self.rotation.w, self.rotation.x, self.rotation.y, self.rotation.z)
            )
            self.mesh.apply_transform(rotation_matrix)

        # If position is provided, translate mesh.
        if self.position is not None:
            position_matrix = translation_matrix(self.position)
            self.mesh.apply_transform(position_matrix)

        self.orientation = orientation

    @classmethod
    def fromFile(
        cls, path, filetype=None, compressed=None, binary=False, unify=True, **kwargs
    ):
        """Load a mesh region from a file, attempting to infer filetype and compression.

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
            kwargs: Additional arguments to the MeshRegion initializer.
        """
        mesh = loadMesh(path, filetype, compressed, binary)

        if unify and issubclass(cls, MeshVolumeRegion):
            mesh = unifyMesh(mesh, verbose=True)

        return cls(mesh=mesh, **kwargs)

    ## Lazy Construction Methods ##
    def sampleGiven(self, value):
        if isinstance(self, MeshVolumeRegion):
            cls = MeshVolumeRegion
        elif isinstance(self, MeshSurfaceRegion):
            cls = MeshSurfaceRegion
        else:
            assert False

        return cls(
            mesh=value[self._mesh],
            dimensions=value[self.dimensions],
            position=value[self.position],
            rotation=value[self.rotation],
            orientation=value[self.orientation],
            tolerance=self.tolerance,
            centerMesh=self.centerMesh,
            onDirection=self.onDirection,
            engine=self.engine,
            name=self.name,
        )

    def evaluateInner(self, context):
        if isinstance(self, MeshVolumeRegion):
            cls = MeshVolumeRegion
        elif isinstance(self, MeshSurfaceRegion):
            cls = MeshSurfaceRegion
        else:
            assert False

        mesh = valueInContext(self._mesh, context)
        dimensions = valueInContext(self.dimensions, context)
        position = valueInContext(self.position, context)
        rotation = valueInContext(self.rotation, context)
        orientation = valueInContext(self.orientation, context)

        return cls(
            mesh,
            dimensions,
            position,
            rotation,
            orientation,
            tolerance=self.tolerance,
            centerMesh=self.centerMesh,
            onDirection=self.onDirection,
            engine=self.engine,
            name=self.name,
        )

    ## API Methods ##
    @property
    @distributionFunction
    def mesh(self):
        return self._mesh

    @distributionFunction
    def projectVector(self, point, onDirection):
        """Find the nearest point in the region following the onDirection or its negation.

        Returns None if no such points exist.
        """
        # Check if this region contains the point, in which case we simply
        # return the point.
        if self.containsPoint(point):
            return point

        # Get first point hit in both directions of ray
        point = point.coordinates

        if onDirection is not None:
            onDirection = numpy.array(onDirection)
        else:
            if isinstance(self, MeshVolumeRegion):
                onDirection = numpy.array((0, 0, 1))
            elif isinstance(self, MeshSurfaceRegion):
                onDirection = (
                    sum(self.mesh.face_normals * self.mesh.area_faces[:, numpy.newaxis])
                    / self.mesh.area
                )
                onDirection = onDirection / numpy.linalg.norm(onDirection)

        intersection_data, _, _ = self.mesh.ray.intersects_location(
            ray_origins=[point, point],
            ray_directions=[onDirection, -1 * onDirection],
            multiple_hits=False,
        )

        if len(intersection_data) == 0:
            return None

        distances = numpy.linalg.norm(intersection_data - numpy.asarray(point))
        closest_point = intersection_data[numpy.argmin(distances)]

        return Vector(*closest_point)

    @cached_property
    @distributionFunction
    def circumcircle(self):
        """Compute an upper bound on the radius of the region"""
        center_point = Vector(*self.mesh.bounding_box.center_mass)
        half_extents = [val / 2 for val in self.mesh.extents]
        circumradius = hypot(*half_extents)

        return (center_point, circumradius)

    @cached_property
    def isConvex(self):
        return self.mesh.is_convex

    @property
    def AABB(self):
        return (
            tuple(self.mesh.bounds[0]),
            tuple(self.mesh.bounds[1]),
            tuple(self.mesh.bounds[2]),
        )

    @cached_property
    def _boundingPolygon(self):
        assert not isLazy(self)

        # Relatively fast case for convex regions
        if self.isConvex:
            return shapely.geometry.MultiPoint(self.mesh.vertices).convex_hull

        # Generic case for arbitrary shapes
        if self.mesh.is_watertight:
            projection = trimesh.path.polygons.projected(
                self.mesh, normal=(0, 0, 1), rpad=1e-4
            )
        else:
            # Special parameters to use all faces if mesh is not watertight.
            projection = trimesh.path.polygons.projected(
                self.mesh,
                normal=(0, 0, 1),
                rpad=1e-4,
                ignore_sign=False,
                tol_dot=-float("inf"),
            )

        return projection

    @cached_property
    @distributionFunction
    def boundingPolygon(self):
        """A PolygonalRegion bounding the mesh"""
        return PolygonalRegion(polygon=self._boundingPolygon)

    def __getstate__(self):
        state = self.__dict__.copy()
        # Make copy of mesh to clear non-picklable cache
        state["_mesh"] = self._mesh.copy()
        return state


class MeshVolumeRegion(MeshRegion):
    """A region representing the volume of a mesh.

    The mesh passed must be a `trimesh.base.Trimesh` object that represents a well defined
    volume (i.e. the ``is_volume`` property must be true), meaning the mesh must be watertight,
    have consistent winding and have outward facing normals.

    The mesh is first placed so the origin is at the center of the bounding box (unless ``centerMesh`` is ``False``).
    The mesh is scaled to ``dimensions``, translated so the center of the bounding box of the mesh is at ``positon``,
    and then rotated to ``rotation``.

    Meshes are centered by default (since ``centerMesh`` is true by default). If you disable this operation, do note
    that scaling and rotation transformations may not behave as expected, since they are performed around the origin.

    Args:
        mesh: The base mesh for this region.
        name: An optional name to help with debugging.
        dimensions: An optional 3-tuple, with the values representing width, length, height
          respectively. The mesh will be scaled such that the bounding box for the mesh has
          these dimensions.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for internal computations.
        centerMesh: Whether or not to center the mesh after copying and before transformations.
        onDirection: The direction to use if an object being placed on this region doesn't specify one.
        engine: Which engine to use for mesh operations. Either "blender" or "scad".
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if isLazy(self):
            return

        # Validate dimensions
        if self.dimensions is not None:
            for dim, name in zip(self.dimensions, ("width", "length", "height")):
                if dim <= 0:
                    raise ValueError(f"{name} of MeshVolumeRegion must be positive")

        # Ensure the mesh is a well defined volume
        if not self._mesh.is_volume:
            raise ValueError(
                "A MeshVolumeRegion cannot be defined with a mesh that does not have a well defined volume."
                " Consider using scenic.core.utils.repairMesh."
            )

        # Compute how many samples are necessary to achieve 99% probability
        # of success when rejection sampling volume.
        p_volume = self._mesh.volume / self._mesh.bounding_box.volume

        if p_volume > 0.99:
            self.num_samples = 1
        else:
            self.num_samples = min(1e6, max(1, math.ceil(math.log(0.01, 1 - p_volume))))

        # Always try to take at least 8 samples to avoid surface point total rejections
        self.num_samples = max(self.num_samples, 8)

    # Property testing methods #
    @distributionFunction
    def intersects(self, other, triedReversed=False):
        """Check if this region intersects another.

        This function handles intersect calculations for `MeshVolumeRegion` with:
        * `MeshVolumeRegion`
        * `MeshSurfaceRegion`
        * `PolygonalFootprintRegion`
        """
        if isinstance(other, MeshVolumeRegion):
            # PASS 1
            # Check if bounding boxes intersect. If not, volumes cannot intersect.
            # For bounding boxes to intersect there must be overlap of the bounds
            # in all 3 dimensions.
            range_overlaps = [
                (self.mesh.bounds[0, dim] <= other.mesh.bounds[1, dim])
                and (other.mesh.bounds[0, dim] <= self.mesh.bounds[1, dim])
                for dim in range(3)
            ]
            bb_overlap = all(range_overlaps)

            if not bb_overlap:
                return False

            # PASS 2
            # Compute inradius and circumradius for a candidate point in each region,
            # and compute the inradius and circumradius of each point. If the candidate
            # points are closer than the sum of the inradius values, they must intersect.
            # If the candidate points are farther apart than the sum of the circumradius
            # values, they can't intersect.

            # Get a candidate point from each mesh. If the center of the object is in the mesh use that.
            # Otherwise try to sample a point as a candidate, skipping this pass if the sample fails.
            if self.containsPoint(Vector(*self.mesh.bounding_box.center_mass)):
                s_candidate_point = Vector(*self.mesh.bounding_box.center_mass)
            elif (
                len(samples := trimesh.sample.volume_mesh(self.mesh, self.num_samples))
                > 0
            ):
                s_candidate_point = Vector(*samples[0])
            else:
                s_candidate_point = None

            if other.containsPoint(Vector(*other.mesh.bounding_box.center_mass)):
                o_candidate_point = Vector(*other.mesh.bounding_box.center_mass)
            elif (
                len(samples := trimesh.sample.volume_mesh(other.mesh, other.num_samples))
                > 0
            ):
                o_candidate_point = Vector(*samples[0])
            else:
                o_candidate_point = None

            if s_candidate_point is not None and o_candidate_point is not None:
                # Compute the inradius of each object from its candidate point.
                s_inradius = abs(
                    trimesh.proximity.ProximityQuery(self.mesh).signed_distance(
                        [s_candidate_point]
                    )[0]
                )
                o_inradius = abs(
                    trimesh.proximity.ProximityQuery(other.mesh).signed_distance(
                        [o_candidate_point]
                    )[0]
                )

                # Compute the circumradius of each object from its candidate point.
                s_circumradius = numpy.max(
                    numpy.linalg.norm(self.mesh.vertices - s_candidate_point, axis=1)
                )
                o_circumradius = numpy.max(
                    numpy.linalg.norm(other.mesh.vertices - o_candidate_point, axis=1)
                )

                # Get the distance between the two points and check for mandatory or impossible collision.
                point_distance = s_candidate_point.distanceTo(o_candidate_point)

                if point_distance < s_inradius + o_inradius:
                    return True

                if point_distance > s_circumradius + o_circumradius:
                    return False

            # PASS 3
            # Use Trimesh's collision manager to check for intersection.
            # If the surfaces collide, that implies a collision of the volumes.
            # Cheaper than computing volumes immediately.
            collision_manager = trimesh.collision.CollisionManager()

            collision_manager.add_object("SelfRegion", self.mesh)
            collision_manager.add_object("OtherRegion", other.mesh)

            surface_collision = collision_manager.in_collision_internal()

            if surface_collision:
                return True

            if self.mesh.is_convex and other.mesh.is_convex:
                # For convex shapes, the manager detects containment as well as
                # surface intersections, so we can just return the result
                return surface_collision

            # PASS 4
            # If we have 2 candidate points and both regions have only one body,
            # we can just check if either region contains the candidate point of the
            # other. (This is because we previously ruled out surface intersections)
            if (
                s_candidate_point is not None
                and o_candidate_point is not None
                and self.mesh.body_count == 1
                and other.mesh.body_count == 1
            ):
                return self.containsPoint(o_candidate_point) or other.containsPoint(
                    s_candidate_point
                )

            # PASS 5
            # Compute intersection and check if it's empty. Expensive but guaranteed
            # to give the right answer.
            return not isinstance(self.intersect(other), EmptyRegion)

        if isinstance(other, MeshSurfaceRegion):
            # PASS 1
            # Check if bounding boxes intersect. If not, volumes cannot intersect.
            # For bounding boxes to intersect there must be overlap of the bounds
            # in all 3 dimensions.
            range_overlaps = [
                (self.mesh.bounds[0, dim] <= other.mesh.bounds[1, dim])
                and (other.mesh.bounds[0, dim] <= self.mesh.bounds[1, dim])
                for dim in range(3)
            ]
            bb_overlap = all(range_overlaps)

            if not bb_overlap:
                return False

            # PASS 2
            # Use Trimesh's collision manager to check for intersection.
            # If the surfaces collide (or surface is contained in the mesh),
            # that implies a collision of the volumes. Cheaper than computing
            # intersection. Must use a SurfaceCollisionTrimesh object for the surface
            # mesh to ensure that a collision implies surfaces touching.
            collision_manager = trimesh.collision.CollisionManager()

            collision_manager.add_object("SelfRegion", self.mesh)
            collision_manager.add_object(
                "OtherRegion",
                SurfaceCollisionTrimesh(
                    faces=other.mesh.faces, vertices=other.mesh.vertices
                ),
            )

            surface_collision = collision_manager.in_collision_internal()

            if surface_collision:
                return True

            # PASS 3
            # There is no surface collision, so all points are in/out. Check the first
            # point and return that.
            return self.containsPoint(other.mesh.vertices[0])

        if isinstance(other, PolygonalFootprintRegion):
            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            centerZ = (vertical_bounds[1] + vertical_bounds[0]) / 2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(centerZ, mesh_height)

            return self.intersects(bounded_footprint)

        return super().intersects(other, triedReversed)

    @distributionFunction
    def containsPoint(self, point):
        """Check if this region's volume contains a point."""
        return self.distanceTo(point) <= self.tolerance

    @distributionFunction
    def containsObject(self, obj):
        """Check if this region's volume contains an :obj:`~scenic.core.object_types.Object`."""
        # PASS 1
        # Check if bounding boxes intersect. If not, volumes cannot intersect and so
        # the object cannot be contained in this region.
        range_overlaps = [
            (self.mesh.bounds[0, dim] <= obj.occupiedSpace.mesh.bounds[1, dim])
            and (obj.occupiedSpace.mesh.bounds[0, dim] <= self.mesh.bounds[1, dim])
            for dim in range(3)
        ]
        bb_overlap = all(range_overlaps)

        if not bb_overlap:
            return False

        # PASS 2
        # If this region is convex, first check if we contain all corners of the object's bounding box.
        # If so, return True. Otherwise, check if all points are contained and return that value.
        if self.isConvex:
            pq = trimesh.proximity.ProximityQuery(self.mesh)
            bb_distances = pq.signed_distance(obj.boundingBox.mesh.vertices)

            if numpy.all(bb_distances > 0):
                return True

            vertex_distances = pq.signed_distance(obj.occupiedSpace.mesh.vertices)

            return numpy.all(vertex_distances > 0)

        # PASS 3
        # Take the object's position if contained in the mesh, or a random sample otherwise.
        # Then check if the point is not in the region, return False if so. Otherwise, compute
        # the circumradius of the object from that point and see if the closest point on the
        # mesh is farther than the circumradius. If it is, then the object must be contained and
        # return True.

        # Get a candidate point from the object mesh. If the position of the object is in the mesh use that.
        # Otherwise try to sample a point as a candidate, skipping this pass if the sample fails.
        if obj.containsPoint(obj.position):
            obj_candidate_point = obj.position
        elif (
            len(
                samples := trimesh.sample.volume_mesh(
                    obj.occupiedSpace.mesh, obj.occupiedSpace.num_samples
                )
            )
            > 0
        ):
            obj_candidate_point = Vector(*samples[0])
        else:
            obj_candidate_point = None

        if obj_candidate_point is not None:
            # If this region doesn't contain the candidate point, it can't contain the object.
            if not self.containsPoint(obj_candidate_point):
                return False

            # Compute the circumradius of the object from the candidate point.
            obj_circumradius = numpy.max(
                numpy.linalg.norm(
                    obj.occupiedSpace.mesh.vertices - obj_candidate_point, axis=1
                )
            )

            # Compute the minimum distance from the region to this point.
            pq = trimesh.proximity.ProximityQuery(self.mesh)
            region_distance = abs(pq.signed_distance([obj_candidate_point])[0])

            if region_distance > obj_circumradius:
                return True

        # PASS 4
        # Take the region's center_mass if contained in the mesh, or a random sample otherwise.
        # Then get the circumradius of the region from that point and the farthest point on
        # the object from this point. If the maximum distance is greater than the circumradius,
        # return False.

        # Get a candidate point from the rgion mesh. If the center of mass of the region is in the mesh use that.
        # Otherwise try to sample a point as a candidate, skipping this pass if the sample fails.
        if self.containsPoint(Vector(*self.mesh.bounding_box.center_mass)):
            reg_candidate_point = Vector(*self.mesh.bounding_box.center_mass)
        elif len(samples := trimesh.sample.volume_mesh(self.mesh, self.num_samples)) > 0:
            reg_candidate_point = Vector(*samples[0])
        else:
            reg_candidate_point = None

        if reg_candidate_point is not None:
            # Calculate circumradius of the region from the candidate_point
            reg_circumradius = numpy.max(
                numpy.linalg.norm(self.mesh.vertices - reg_candidate_point, axis=1)
            )

            # Calculate maximum distance to the object.
            obj_max_distance = numpy.max(
                numpy.linalg.norm(
                    obj.occupiedSpace.mesh.vertices - reg_candidate_point, axis=1
                )
            )

            if obj_max_distance > reg_circumradius:
                return False

        # PASS 5
        # If the difference between the object's region and this region is empty,
        # i.e. obj_region - self_region = EmptyRegion, that means the object is
        # entirely contained in this region.
        diff_region = obj.occupiedSpace.difference(self)

        return isinstance(diff_region, EmptyRegion)

    def containsRegionInner(self, reg, tolerance):
        if tolerance != 0:
            warnings.warn(
                "Nonzero tolerances are ignored for containsRegionInner on MeshVolumeRegion"
            )

        if isinstance(reg, MeshVolumeRegion):
            diff_region = reg.difference(self)

            return isinstance(diff_region, EmptyRegion)

        raise NotImplementedError

    # Composition methods #
    @cached_method
    def intersect(self, other, triedReversed=False):
        """Get a `Region` representing the intersection of this region with another.

        This function handles intersection computation for `MeshVolumeRegion` with:
        * `MeshVolumeRegion`
        * `PolygonalFootprintRegion`
        * `PolygonalRegion`
        * `PathRegion`
        * `PolylineRegion`
        """
        # If one of the regions isn't fixed, fall back on default behavior
        if isLazy(self) or isLazy(other):
            return super().intersect(other, triedReversed)

        if isinstance(other, MeshVolumeRegion):
            # Other region is a mesh volume. We can extract the mesh to perform boolean operations on it
            other_mesh = other.mesh

            # Compute intersection using Trimesh
            try:
                new_mesh = self.mesh.intersection(other_mesh, engine=self.engine)
            except ValueError as exc:
                raise ValueError(
                    "Unable to compute mesh boolean operation. Do you have the Blender and OpenSCAD installed on your system?"
                ) from exc

            if new_mesh.is_empty:
                return nowhere
            elif new_mesh.is_volume:
                return MeshVolumeRegion(
                    new_mesh,
                    tolerance=min(self.tolerance, other.tolerance),
                    centerMesh=False,
                    engine=self.engine,
                )
            else:
                # Something went wrong, abort
                return super().intersect(other, triedReversed)

        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a polygonal footprint region. We can bound it in the vertical dimension
            # and then calculate the intersection with the resulting mesh volume.

            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            centerZ = (vertical_bounds[1] + vertical_bounds[0]) / 2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(centerZ, mesh_height)

            return self.intersect(bounded_footprint)

        if isinstance(other, PolygonalRegion):
            # Other region can be represented by a polygon. We can slice the volume at the polygon's height,
            # and then take the intersection of the resulting polygons.
            origin_point = (self.mesh.centroid[0], self.mesh.centroid[1], other.z)
            slice_3d = self.mesh.section(
                plane_origin=origin_point, plane_normal=[0, 0, 1]
            )

            if slice_3d is None:
                return nowhere

            slice_2d, _ = slice_3d.to_planar(to_2D=numpy.eye(4))
            polygons = MultiPolygon(slice_2d.polygons_full) & other.polygons

            if polygons.is_empty:
                return nowhere

            return PolygonalRegion(polygon=polygons, z=other.z)

        if isinstance(other, PathRegion):
            # Other region is one or more 2d line segments. We can divide each line segment into pieces that are entirely inside/outside
            # the mesh. Then we can construct a new Polyline region using only the line segments that are entirely inside.

            # Extract lines from region
            edges = [
                (other.vert_to_vec[v1], other.vert_to_vec[v2]) for v1, v2 in other.edges
            ]

            # Split lines anytime they cross the mesh boundaries
            refined_polylines = []

            for line_iter, line in enumerate(edges):
                source, dest = line

                ray = dest - source
                ray = ray / numpy.linalg.norm(ray)

                intersections = self.mesh.ray.intersects_location(
                    ray_origins=[source], ray_directions=[ray]
                )[0]

                inner_points = sorted(
                    intersections, key=lambda pos: numpy.linalg.norm(source - pos)
                )
                inner_points = filter(
                    lambda point: numpy.linalg.norm(point - source)
                    < numpy.linalg.norm(dest - source),
                    inner_points,
                )

                refined_points = [source] + list(inner_points) + [dest]

                refined_polylines.append(refined_points)

            # Keep only lines and vertices for line segments in the mesh.
            internal_lines = []

            for polyline in refined_polylines:
                source = polyline[0]

                for pt_iter in range(1, len(polyline)):
                    dest = polyline[pt_iter]

                    midpoint = (source + dest) / 2
                    if self.containsPoint(midpoint):
                        internal_lines.append((source, dest))

                    source = dest

            # Check if merged lines is empty. If so, return the EmptyRegion. Otherwise,
            # transform merged lines back into a path region.
            if internal_lines:
                return PathRegion(polylines=internal_lines)
            else:
                return nowhere

        if isinstance(other, PolylineRegion):
            # Other region is one or more 2d line segments. We can divide each line segment into pieces that are entirely inside/outside
            # the mesh. Then we can construct a new Polyline region using only the line segments that are entirely inside.

            other_polygon = toPolygon(other)

            # Extract a list of the points defining the line segments.
            if isinstance(other_polygon, shapely.geometry.linestring.LineString):
                points_lists = [other_polygon.coords]
            else:
                points_lists = [ls.coords for ls in other_polygon.geoms]

            # Extract lines from region
            lines = []
            for point_list in points_lists:
                vertices = [numpy.array(toVector(coords)) for coords in point_list]
                segments = [(v_iter - 1, v_iter) for v_iter in range(1, len(vertices))]

                lines.append((vertices, segments))

            # Split lines anytime they cross the mesh boundaries
            refined_lines = []

            for vertices, segments in lines:
                refined_vertices = []

                for line_iter, line in enumerate(segments):
                    source = vertices[line[0]]
                    dest = vertices[line[1]]
                    ray = dest - source

                    if line_iter == 0:
                        refined_vertices.append(source)

                    ray = ray / numpy.linalg.norm(ray)

                    intersections = self.mesh.ray.intersects_location(
                        ray_origins=[source], ray_directions=[ray]
                    )[0]

                    inner_points = sorted(
                        intersections, key=lambda pos: numpy.linalg.norm(source - pos)
                    )

                    for point in inner_points:
                        if numpy.linalg.norm(point - source) < numpy.linalg.norm(
                            dest - source
                        ):
                            refined_vertices.append(point)

                    refined_vertices.append(dest)

                refined_segments = [
                    (v_iter - 1, v_iter) for v_iter in range(1, len(refined_vertices))
                ]

                refined_lines.append((refined_vertices, refined_segments))

            # Keep only lines and vertices for line segments in the mesh. Also converts them
            # to shapely's point format.
            internal_lines = []

            for vertices, segments in refined_lines:
                for segment in segments:
                    source = vertices[segment[0]]
                    dest = vertices[segment[1]]

                    midpoint = (source + dest) / 2

                    if self.containsPoint(midpoint):
                        internal_lines.append((source, dest))

            merged_lines = shapely.ops.linemerge(internal_lines)

            # Check if merged lines is empty. If so, return the EmptyRegion. Otherwise,
            # transform merged lines back into a polyline region.
            if merged_lines:
                return PolylineRegion(polyline=shapely.ops.linemerge(internal_lines))
            else:
                return nowhere

        # Don't know how to compute this intersection, fall back to default behavior.
        return super().intersect(other, triedReversed)

    def union(self, other, triedReversed=False):
        """Get a `Region` representing the union of this region with another.

        This function handles union computation for `MeshVolumeRegion` with:
            - `MeshVolumeRegion`
        """
        # If one of the regions isn't fixed, fall back on default behavior
        if isLazy(self) or isLazy(other):
            return super().union(other, triedReversed)

        # If other region is represented by a mesh, we can extract the mesh to
        # perform boolean operations on it
        if isinstance(other, MeshVolumeRegion):
            other_mesh = other.mesh

            # Compute union using Trimesh
            try:
                new_mesh = self.mesh.union(other_mesh, engine=self.engine)
            except ValueError as exc:
                raise ValueError(
                    "Unable to compute mesh boolean operation. Do you have the Blender and OpenSCAD installed on your system?"
                ) from exc

            if new_mesh.is_empty:
                return nowhere
            elif new_mesh.is_volume:
                return MeshVolumeRegion(
                    new_mesh,
                    tolerance=min(self.tolerance, other.tolerance),
                    centerMesh=False,
                    engine=self.engine,
                )
            else:
                # Something went wrong, abort
                return super().union(other, triedReversed)

        # Don't know how to compute this union, fall back to default behavior.
        return super().union(other, triedReversed)

    def difference(self, other, debug=False):
        """Get a `Region` representing the difference of this region with another.

        This function handles union computation for `MeshVolumeRegion` with:
        * `MeshVolumeRegion`
        * `PolygonalFootprintRegion`
        """
        # If one of the regions isn't fixed, fall back on default behavior
        if isLazy(self) or isLazy(other):
            return super().difference(other)

        # If other region is represented by a mesh, we can extract the mesh to
        # perform boolean operations on it
        if isinstance(other, MeshVolumeRegion):
            other_mesh = other.mesh

            # Compute difference using Trimesh
            try:
                new_mesh = self.mesh.difference(
                    other_mesh, engine=self.engine, debug=debug
                )
            except ValueError as exc:
                raise ValueError(
                    "Unable to compute mesh boolean operation. Do you have the Blender and OpenSCAD installed on your system?"
                ) from exc

            if new_mesh.is_empty:
                return nowhere
            elif new_mesh.is_volume:
                return MeshVolumeRegion(
                    new_mesh,
                    tolerance=min(self.tolerance, other.tolerance),
                    centerMesh=False,
                    engine=self.engine,
                )
            else:
                # Something went wrong, abort
                return super().difference(other)

        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a polygonal footprint region. We can bound it in the vertical dimension
            # and then calculate the difference with the resulting mesh volume.

            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            centerZ = (vertical_bounds[1] + vertical_bounds[0]) / 2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(centerZ, mesh_height)

            return self.difference(bounded_footprint)

        # Don't know how to compute this difference, fall back to default behavior.
        return super().difference(other)

    def uniformPointInner(self):
        # TODO: Look into tetrahedralization, perhaps to be turned on when a heuristic
        # is met. Currently using Trimesh's rejection sampling.
        sample = trimesh.sample.volume_mesh(self.mesh, self.num_samples)

        if len(sample) == 0:
            raise RejectionException("Rejection sampling MeshVolumeRegion failed.")
        else:
            return Vector(*sample[0])

    @distributionFunction
    def distanceTo(self, point):
        """Get the minimum distance from this region to the specified point."""
        point = toVector(point, f"Could not convert {point} to vector.")

        pq = trimesh.proximity.ProximityQuery(self.mesh)

        dist = pq.signed_distance([point.coordinates])[0]

        # Positive distance indicates being contained in the mesh.
        if dist > 0:
            dist = 0

        return abs(dist)

    @cached_property
    def inradius(self):
        center_point = self.mesh.bounding_box.center_mass

        pq = trimesh.proximity.ProximityQuery(self.mesh)
        region_distance = abs(pq.signed_distance([center_point])[0])

        if region_distance < 0:
            return 0
        else:
            return region_distance

    @property
    def dimensionality(self):
        return 3

    @cached_property
    def size(self):
        return self.mesh.mass / self.mesh.density

    ## Utility Methods ##
    @cached_method
    def getSurfaceRegion(self):
        """Return a region equivalent to this one, except as a MeshSurfaceRegion"""
        return MeshSurfaceRegion(
            self.mesh,
            self.name,
            orientation=self.orientation,
            tolerance=self.tolerance,
            centerMesh=False,
            onDirection=self.onDirection,
            engine=self.engine,
        )

    def getVolumeRegion(self):
        """Returns this object, as it is already a MeshVolumeRegion"""
        return self


class MeshSurfaceRegion(MeshRegion):
    """A region representing the surface of a mesh.

    The mesh is first placed so the origin is at the center of the bounding box (unless ``centerMesh`` is ``False``).
    The mesh is scaled to ``dimensions``, translated so the center of the bounding box of the mesh is at ``positon``,
    and then rotated to ``rotation``.

    Meshes are centered by default (since ``centerMesh`` is true by default). If you disable this operation, do note
    that scaling and rotation transformations may not behave as expected, since they are performed around the origin.

    If an orientation is not passed to this mesh, a default orientation is provided which provides an orientation
    that aligns an object's z axis with the normal vector of the face containing that point, and has a yaw aligned
    with a yaw of 0 in the global coordinate system.

    Args:
        mesh: The base mesh for this region.
        name: An optional name to help with debugging.
        dimensions: An optional 3-tuple, with the values representing width, length, height respectively.
          The mesh will be scaled such that the bounding box for the mesh has these dimensions.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for internal computations.
        centerMesh: Whether or not to center the mesh after copying and before transformations.
        onDirection: The direction to use if an object being placed on this region doesn't specify one.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Validate dimensions
        if self.dimensions is not None:
            for dim, name in zip(self.dimensions, ("width", "length", "height")):
                if dim < 0:
                    raise ValueError(f"{name} of MeshSurfaceRegion must be nonnegative")

        # Set default orientation to one inferred from face norms if none is provided.
        if self.orientation is None:
            self.orientation = VectorField(
                "DefaultSurfaceVectorField", lambda pos: self.getFlatOrientation(pos)
            )

    # Property testing methods #
    @distributionFunction
    def intersects(self, other, triedReversed=False):
        """Check if this region's surface intersects another.

        This function handles intersection computation for `MeshSurfaceRegion` with:
        * `MeshSurfaceRegion`
        * `PolygonalFootprintRegion`
        """
        if isinstance(other, MeshSurfaceRegion):
            # Uses Trimesh's collision manager to check for intersection of the
            # surfaces. Use SurfaceCollisionTrimesh objects to ensure collisions
            # actually imply a surface collision.
            collision_manager = trimesh.collision.CollisionManager()

            collision_manager.add_object(
                "SelfRegion",
                SurfaceCollisionTrimesh(
                    faces=self.mesh.faces, vertices=self.mesh.vertices
                ),
            )
            collision_manager.add_object(
                "OtherRegion",
                SurfaceCollisionTrimesh(
                    faces=other.mesh.faces, vertices=other.mesh.vertices
                ),
            )

            surface_collision = collision_manager.in_collision_internal()

            return surface_collision

        if isinstance(other, PolygonalFootprintRegion):
            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            centerZ = (vertical_bounds[1] + vertical_bounds[0]) / 2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(centerZ, mesh_height)

            return self.intersects(bounded_footprint)

        return super().intersects(other, triedReversed)

    @distributionFunction
    def containsPoint(self, point):
        """Check if this region's surface contains a point."""
        # First compute the minimum distance to the point.
        min_distance = self.distanceTo(point)

        # If the minimum distance is within tolerance of 0, the mesh contains the point.
        return min_distance < self.tolerance

    def containsObject(self, obj):
        # A surface cannot contain an object, which must have a volume.
        return False

    def containsRegionInner(self, reg, tolerance):
        if tolerance != 0:
            warnings.warn(
                "Nonzero tolerances are ignored for containsRegionInner on MeshSurfaceRegion"
            )

        if isinstance(reg, MeshSurfaceRegion):
            diff_region = reg.difference(self)

            return isinstance(diff_region, EmptyRegion)

        raise NotImplementedError

    def uniformPointInner(self):
        return Vector(*trimesh.sample.sample_surface(self.mesh, 1)[0][0])

    @distributionFunction
    def distanceTo(self, point):
        """Get the minimum distance from this object to the specified point."""
        point = toVector(point, f"Could not convert {point} to vector.")

        pq = trimesh.proximity.ProximityQuery(self.mesh)

        dist = abs(pq.signed_distance([point.coordinates])[0])

        return dist

    @property
    def dimensionality(self):
        return 2

    @cached_property
    def size(self):
        return self.mesh.area

    @distributionMethod
    def getFlatOrientation(self, pos):
        """Get a flat orientation at a point in the region.

        Given a point on the surface of the mesh, returns an orientation that aligns
        an instance's z axis with the normal vector of the face containing that point.
        Since there are infinitely many such orientations, the orientation returned
        has yaw aligned with a global yaw of 0.

        If ``pos`` is not within ``self.tolerance`` of the surface of the mesh, a
        ``RejectionException`` is raised.
        """
        prox_query = trimesh.proximity.ProximityQuery(self.mesh)
        _, distance, triangle_id = prox_query.on_surface([pos.coordinates])
        if distance > self.tolerance:
            raise RejectionException(
                "Attempted to get flat orientation of a mesh away from a surface."
            )
        face_normal_vector = self.mesh.face_normals[triangle_id][0]

        # Get arbitrary orientation that aligns object with face_normal_vector
        transform = trimesh.geometry.align_vectors([0, 0, 1], face_normal_vector)
        base_orientation = Orientation(
            scipy.spatial.transform.Rotation.from_matrix(transform[:3, :3])
        )

        # Add yaw offset to orientation to align object's yaw with global 0
        yaw_offset = (
            Vector(0, 1, 0)
            .applyRotation(base_orientation.inverse)
            .sphericalCoordinates()[1]
        )
        final_orientation = base_orientation * Orientation._fromHeading(yaw_offset)

        return final_orientation

    ## Utility Methods ##
    @cached_method
    def getVolumeRegion(self):
        """Return a region equivalent to this one, except as a MeshVolumeRegion"""
        return MeshVolumeRegion(
            self.mesh,
            self.name,
            orientation=self.orientation,
            tolerance=self.tolerance,
            centerMesh=False,
            onDirection=self.onDirection,
            engine=self.engine,
        )

    def getSurfaceRegion(self):
        """Returns this object, as it is already a MeshSurfaceRegion"""
        return self


class BoxRegion(MeshVolumeRegion):
    """Region in the shape of a rectangular cuboid, i.e. a box.

    By default the unit box centered at the origin and aligned with the axes is used.

    Parameters are the same as `MeshVolumeRegion`, with the exception of the ``mesh``
    parameter which is excluded.
    """

    def __init__(self, *args, **kwargs):
        box_mesh = trimesh.creation.box((1, 1, 1))
        super().__init__(mesh=box_mesh, *args, **kwargs)

    @cached_property
    def isConvex(self):
        return True

    def sampleGiven(self, value):
        return BoxRegion(
            dimensions=value[self.dimensions],
            position=value[self.position],
            rotation=value[self.rotation],
            orientation=value[self.orientation],
            tolerance=self.tolerance,
            engine=self.engine,
            name=self.name,
        )

    def evaluateInner(self, context):
        dimensions = valueInContext(self.dimensions, context)
        position = valueInContext(self.position, context)
        rotation = valueInContext(self.rotation, context)
        orientation = valueInContext(self.orientation, context)

        return BoxRegion(
            dimensions=dimensions,
            position=position,
            rotation=rotation,
            orientation=orientation,
            tolerance=self.tolerance,
            engine=self.engine,
            name=self.name,
        )


class SpheroidRegion(MeshVolumeRegion):
    """Region in the shape of a spheroid.

    By default the unit sphere centered at the origin and aligned with the axes is used.

    Parameters are the same as `MeshVolumeRegion`, with the exception of the ``mesh``
    parameter which is excluded.
    """

    def __init__(self, *args, **kwargs):
        sphere_mesh = trimesh.creation.icosphere(radius=1)
        super().__init__(mesh=sphere_mesh, *args, **kwargs)

    @cached_property
    def isConvex(self):
        return True

    def sampleGiven(self, value):
        return SpheroidRegion(
            dimensions=value[self.dimensions],
            position=value[self.position],
            rotation=value[self.rotation],
            orientation=value[self.orientation],
            tolerance=self.tolerance,
            engine=self.engine,
            name=self.name,
        )

    def evaluateInner(self, context):
        dimensions = valueInContext(self.dimensions, context)
        position = valueInContext(self.position, context)
        rotation = valueInContext(self.rotation, context)
        orientation = valueInContext(self.orientation, context)

        return SpheroidRegion(
            dimensions=dimensions,
            position=position,
            rotation=rotation,
            orientation=orientation,
            tolerance=self.tolerance,
            engine=self.engine,
            name=self.name,
        )


class PolygonalFootprintRegion(Region):
    """Region that contains all points in a polygonal footprint, regardless of their z value.

    This region cannot be sampled from, as it has infinite height and therefore infinite volume.

    Args:
        polygon: A ``shapely`` ``Polygon`` or ``MultiPolygon``, that defines the footprint of this region.
        name: An optional name to help with debugging.
    """

    def __init__(self, polygon, name=None):
        if isinstance(polygon, shapely.geometry.Polygon):
            self.polygons = shapely.geometry.MultiPolygon([polygon])
        elif isinstance(polygon, shapely.geometry.MultiPolygon):
            self.polygons = polygon
        else:
            raise TypeError(
                f"tried to create PolygonalFootprintRegion from non-polygon {polygon}"
            )
        shapely.prepare(self.polygons)

        super().__init__(name)
        self._bounded_cache = None

    def intersect(self, other, triedReversed=False):
        """Get a `Region` representing the intersection of this region with another.

        This function handles intersection computation for `PolygonalFootprintRegion` with:
        * `PolygonalFootprintRegion`
        * `PolygonalRegion`
        """
        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a PolygonalFootprintRegion, so we can just intersect the base polygons
            # and take the footprint of the result, if it isn't empty.
            new_poly = self.polygons.intersection(other.polygons)

            if new_poly.is_empty:
                return nowhere
            else:
                return PolygonalFootprintRegion(new_poly)

        if isinstance(other, PolygonalRegion):
            # Other region can be represented by a polygon. We can take the intersection of that
            # polygon with our base polygon at the correct z position, and then output the resulting polygon.
            return PolygonalRegion(polygon=self.polygons, z=other.z).intersect(other)

        if isinstance(other, PathRegion):
            center_z = (other.AABB[2][1] + other.AABB[2][0]) / 2
            height = other.AABB[2][1] - other.AABB[2][0] + 1
            return self.approxBoundFootprint(center_z, height).intersect(other)

        return super().intersect(other, triedReversed)

    def union(self, other, triedReversed=False):
        """Get a `Region` representing the union of this region with another.

        This function handles union computation for `PolygonalFootprintRegion` with:
        * `PolygonalFootprintRegion`
        """
        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a PolygonalFootprintRegion, so we can just union the base polygons
            # and take the footprint of the result.
            return PolygonalFootprintRegion(self.polygons.union(other.polygons))

        return super().union(other, triedReversed)

    def difference(self, other):
        """Get a `Region` representing the difference of this region with another.

        This function handles difference computation for `PolygonalFootprintRegion` with:
        * `PolygonalFootprintRegion`
        """
        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a PolygonalFootprintRegion, so we can just difference the base polygons
            # and take the footprint of the result, if it isn't empty.
            new_poly = self.polygons.difference(other.polygons)

            if new_poly.is_empty:
                return nowhere
            else:
                return PolygonalFootprintRegion(new_poly)

        return super().difference(other)

    def uniformPointInner(self):
        raise UndefinedSamplingException(
            f"Attempted to sample from a PolygonalFootprintRegion, for which uniform sampling is undefined"
        )

    def containsPoint(self, point):
        """Checks if a point is contained in the polygonal footprint.

        Equivalent to checking if the (x, y) values are contained in the polygon.

        Args:
            point: A point to be checked for containment.
        """
        point = toVector(point)
        return shapely.intersects_xy(self.polygons, point.x, point.y)

    def containsObject(self, obj):
        """Checks if an object is contained in the polygonal footprint.

        Args:
            obj: An object to be checked for containment.
        """
        # Check containment using the bounding polygon of the object.
        return self.polygons.contains(obj._boundingPolygon)

    def containsRegionInner(self, reg, tolerance):
        buffered_polygons = self.polygons.buffer(tolerance)

        if isinstance(other, MeshRegion):
            return buffered_polygons.contains(reg._boundingPolygon)

        if isinstance(other, (PolygonalRegion, PolygonalFootprintRegion)):
            return buffered_polygons.contains(reg.polygons)

        raise NotImplementedError

    @distributionMethod
    def distanceTo(self, point):
        """Minimum distance from this polygonal footprint to the target point"""
        point = toVector(point)
        return self.polygons.distance(makeShapelyPoint(point))

    def projectVector(self, point, onDirection):
        raise NotImplementedError(
            f'{type(self).__name__} does not yet support projection using "on"'
        )

    @property
    def AABB(self):
        raise NotImplementedError

    @property
    def dimensionality(self):
        return 3

    @property
    def size(self):
        return float("inf")

    def approxBoundFootprint(self, centerZ, height):
        """Returns an overapproximation of boundFootprint

        Returns a volume that is guaranteed to contain the result of boundFootprint(centerZ, height),
        but may be taller. Used to save time on recomputing boundFootprint.
        """
        if self._bounded_cache is not None:
            # See if we can reuse a previous region
            prev_centerZ, prev_height, prev_bounded_footprint = self._bounded_cache

            if (
                prev_centerZ + prev_height / 2 > centerZ + height / 2
                and prev_centerZ - prev_height / 2 < centerZ - height / 2
            ):
                # Cached region covers requested region, so return that.
                return prev_bounded_footprint

        # Populate cache, and make the height bigger than requested to try to
        # save on future calls.
        padded_height = 100 * max(1, centerZ) * height
        bounded_footprint = self.boundFootprint(centerZ, padded_height)

        self._bounded_cache = centerZ, padded_height, bounded_footprint

        return bounded_footprint

    def boundFootprint(self, centerZ, height):
        """Cap the footprint of the object to a given height, centered at a given z.

        Args:
            centerZ: The resulting mesh will be vertically centered at this height.
            height: The resulting mesh will have this height.
        """
        # Fall back on progressively higher buffering and simplification to
        # get the mesh to be a valid volume
        tol_sizes = [None, 0.00001, 0.0001, 0.001, 0.01, 0.1, 1]

        for tol_size in tol_sizes:
            # If needed, buffer the multipolygon
            if tol_size is None:
                poly_obj = self.polygons
            else:
                poly_obj = self.polygons.buffer(tol_size).simplify(tol_size)

            # Extract a list of the polygon
            if isinstance(poly_obj, shapely.geometry.Polygon):
                polys = [poly_obj]
            else:
                polys = list(poly_obj.geoms)

            # Extrude the polygon to the desired height
            vf = [trimesh.creation.triangulate_polygon(p, engine="earcut") for p in polys]
            v, f = trimesh.util.append_faces([i[0] for i in vf], [i[1] for i in vf])
            polygon_mesh = trimesh.creation.extrude_triangulation(
                vertices=v, faces=f, height=height
            )

            # Translate the polygon mesh to the desired height.
            polygon_mesh.vertices[:, 2] += (
                centerZ - polygon_mesh.bounding_box.center_mass[2]
            )

            # Check if we have a valid volume
            if polygon_mesh.is_volume:
                if tol_size is not None and tol_size >= 0.01:
                    warnings.warn(
                        f"Computing bounded footprint of polygon resulted in invalid volume, "
                        f"but was able to remedy this by buffering/simplifying polygon by {tol_size}"
                    )
                break

        if not polygon_mesh.is_volume:
            raise RuntimeError(
                "Computing bounded footprint of polygon resulted in invalid volume"
            )

        return MeshVolumeRegion(polygon_mesh, centerMesh=False)

    def buffer(self, amount):
        buffered_polygon = self.polygons.buffer(amount)

        return PolygonalFootprintRegion(polygon=buffered_polygon, name=self.name)

    @cached_property
    def isConvex(self):
        return self.polygons.equals(self.polygons.convex_hull)

    @cached
    def __hash__(self):
        return hash((self.polygons, self.orientation))


class PathRegion(Region):
    """A region composed of multiple polylines in 3D space.

    One of points or polylines should be provided.

    Args:
        points: A list of points defining a single polyline.
        polylines: A list of list of points, defining multiple polylines.
        tolerance: Tolerance used internally.
    """

    def __init__(self, points=None, polylines=None, tolerance=1e-8, name=None):
        super().__init__(name)
        # Standardize inputs
        if points is not None and polylines is not None:
            raise ValueError("Both points and polylines passed to PathRegion initializer")

        if points is not None:
            polylines = [points]
        elif polylines is not None:
            pass
        else:
            raise ValueError("PathRegion created without specifying points or polylines")

        # Extract a list of vertices and edges, without duplicates
        self.vec_to_vert = {}
        self.edges = []
        vertex_iter = 0

        for pls in polylines:
            last_pt = None

            for pt in pls:
                # Extract vertex
                cast_pt = toVector(pt)

                if cast_pt not in self.vec_to_vert:
                    self.vec_to_vert[cast_pt] = vertex_iter
                    vertex_iter += 1

                # Extract edge, if not first point
                if last_pt is not None:
                    new_edge = self.vec_to_vert[last_pt], self.vec_to_vert[cast_pt]
                    self.edges.append(new_edge)

                # Update last point
                last_pt = cast_pt

        self.vert_to_vec = tuple(self.vec_to_vert.keys())
        self.vertices = list(
            sorted(self.vec_to_vert.keys(), key=lambda vec: self.vec_to_vert[vec])
        )

        # Extract length of each edge
        self.edge_lengths = []

        for edge in self.edges:
            v1, v2 = edge
            c1, c2 = self.vert_to_vec[v1], self.vert_to_vec[v2]

            self.edge_lengths.append(c1.distanceTo(c2))

        self.tolerance = tolerance

        self._edgeVectorArray = numpy.asarray(
            [list(self.vert_to_vec[a]) + list(self.vert_to_vec[b]) for a, b in self.edges]
        )

        a = self._edgeVectorArray[:, 0:3]
        b = self._edgeVectorArray[:, 3:6]
        self._normalizedEdgeDirections = (b - a) / numpy.reshape(
            numpy.linalg.norm(b - a, axis=1), (len(a), 1)
        )

    def containsPoint(self, point):
        return self.distanceTo(point) < self.tolerance

    def containsObject(self, obj):
        return False

    def containsRegionInner(self, reg, tolerance):
        raise NotImplementedError

    def distanceTo(self, point):
        p = numpy.asarray(toVector(point))
        a = self._edgeVectorArray[:, 0:3]
        b = self._edgeVectorArray[:, 3:6]
        a_min_p = a - p

        d = (b - a) / numpy.linalg.norm(b - a, axis=1).reshape(len(a), 1)

        # Parallel distances from each end point. Negative indicates on the line segment
        a_dist = numpy.sum((a_min_p) * d, axis=1)
        b_dist = numpy.sum((p - b) * d, axis=1)

        # Actual parallel distance is 0 if on the line segment.
        parallel_dist = numpy.amax(
            [a_dist, b_dist, numpy.zeros(len(self._edgeVectorArray))], axis=0
        )
        perp_dist = numpy.linalg.norm(numpy.cross(a_min_p, d), axis=1)

        return numpy.hypot(parallel_dist, perp_dist).min()

    def projectVector(self, point, onDirection):
        raise NotImplementedError

    @cached_property
    def AABB(self):
        return tuple(
            zip(numpy.amin(self.vertices, axis=0), numpy.amax(self.vertices, axis=0))
        )

    def uniformPointInner(self):
        # Pick an edge, weighted by length, and extract its two points
        edge = random.choices(population=self.edges, weights=self.edge_lengths, k=1)[0]
        v1, v2 = edge
        c1, c2 = self.vert_to_vec[v1], self.vert_to_vec[v2]

        # Sample uniformly from the line segment
        sampled_pt = c1 + random.uniform(0, 1) * (c2 - c1)

        return sampled_pt

    @property
    def dimensionality(self):
        return 1

    @cached_property
    def size(self):
        return sum(self.edge_lengths)


###################################################################################################
# 2D Regions
###################################################################################################


class PolygonalRegion(Region):
    """Region given by one or more polygons (possibly with holes) at a fixed z coordinate.

    The region may be specified by giving either a sequence of points defining the
    boundary of the polygon, or a collection of ``shapely`` polygons (a ``Polygon``
    or ``MultiPolygon``).

    Args:
        points: sequence of points making up the boundary of the polygon (or `None` if
            using the **polygon** argument instead).
        polygon: ``shapely`` polygon or collection of polygons (or `None` if using
            the **points** argument instead).
        z: The z coordinate the polygon is located at.
        orientation (`VectorField`; optional): :term:`preferred orientation` to use.
        name (str; optional): name for debugging.
    """

    def __init__(
        self,
        points=None,
        polygon=None,
        z=0,
        orientation=None,
        name=None,
        additionalDeps=[],
    ):
        super().__init__(
            name, points, polygon, z, *additionalDeps, orientation=orientation
        )

        # Store main parameter
        self._points = points
        self._polygon = polygon
        self.z = z

        # If our region isn't fixed yet, then compute other values later
        if isLazy(self):
            return

        if polygon is None and points is None:
            raise ValueError("must specify points or polygon for PolygonalRegion")
        if polygon is None:
            points = tuple(pt[:2] for pt in points)
            if len(points) == 0:
                raise ValueError("tried to create PolygonalRegion from empty point list!")
            self.points = points
            polygon = shapely.geometry.Polygon(points)

        if isinstance(polygon, shapely.geometry.Polygon):
            self._polygons = shapely.geometry.MultiPolygon([polygon])
        elif isinstance(polygon, shapely.geometry.MultiPolygon):
            self._polygons = polygon
        else:
            raise TypeError(f"tried to create PolygonalRegion from non-polygon {polygon}")

        assert self._polygons

        if not self.polygons.is_valid:
            raise ValueError(
                "tried to create PolygonalRegion with " f"invalid polygon {self.polygons}"
            )

        if (
            points is None
            and len(self.polygons.geoms) == 1
            and len(self.polygons.geoms[0].interiors) == 0
        ):
            self.points = tuple(self.polygons.geoms[0].exterior.coords[:-1])

        if self.polygons.is_empty:
            raise ValueError("tried to create empty PolygonalRegion")
        shapely.prepare(self.polygons)

    ## Lazy Construction Methods ##
    def sampleGiven(self, value):
        return PolygonalRegion(
            points=value[self._points],
            polygon=value[self._polygon],
            z=value[self.z],
            orientation=value[self.orientation],
            name=self.name,
        )

    def evaluateInner(self, context):
        points = valueInContext(self._points, context)
        polygon = valueInContext(self._polygon, context)
        z = valueInContext(self.z, context)
        orientation = valueInContext(self.orientation, context)
        return PolygonalRegion(
            points=points, polygon=polygon, z=z, orientation=orientation, name=self.name
        )

    @property
    @distributionFunction
    def polygons(self):
        return self._polygons

    @cached_property
    @distributionFunction
    def footprint(self):
        return PolygonalFootprintRegion(self.polygons)

    @distributionFunction
    def containsPoint(self, point):
        return self.footprint.containsPoint(point)

    @distributionFunction
    def containsObject(self, obj):
        return self.footprint.containsObject(obj)

    @cached_property
    def _samplingData(self):
        triangles = []
        for polygon in self.polygons.geoms:
            triangles.extend(triangulatePolygon(polygon))
        assert len(triangles) > 0, self.polygons
        shapely.prepare(triangles)
        trianglesAndBounds = tuple((tri, tri.bounds) for tri in triangles)
        areas = (triangle.area for triangle in triangles)
        cumulativeTriangleAreas = tuple(itertools.accumulate(areas))
        return trianglesAndBounds, cumulativeTriangleAreas

    def uniformPointInner(self):
        trisAndBounds, cumulativeAreas = self._samplingData
        triangle, bounds = random.choices(trisAndBounds, cum_weights=cumulativeAreas)[0]
        minx, miny, maxx, maxy = bounds
        # TODO improve?
        while True:
            x, y = random.uniform(minx, maxx), random.uniform(miny, maxy)
            if shapely.intersects_xy(triangle, x, y):
                return self.orient(Vector(x, y, self.z))

    @distributionFunction
    def intersects(self, other, triedReversed=False):
        if isinstance(other, PolygonalRegion):
            if self.z != other.z:
                return False

        poly = toPolygon(other)
        if poly is not None:
            return self.polygons.intersects(poly)

        return super().intersects(other, triedReversed)

    def intersect(self, other, triedReversed=False):
        # If one of the regions isn't fixed, fall back on default behavior
        if isLazy(self) or isLazy(other):
            return super().intersect(other, triedReversed)

        if isinstance(other, PolygonalRegion):
            if self.z != other.z:
                return nowhere

        poly = toPolygon(other)
        orientation = other.orientation if self.orientation is None else self.orientation
        if poly is not None:
            intersection = self.polygons & poly
            if isinstance(intersection, shapely.geometry.GeometryCollection):
                if intersection.area > 0:
                    poly_geoms = (shapely.geometry.Polygon, shapely.geometry.MultiPolygon)
                    geoms = [
                        geom
                        for geom in intersection.geoms
                        if isinstance(geom, poly_geoms)
                    ]
                elif intersection.length > 0:
                    line_geoms = (
                        shapely.geometry.LineString,
                        shapely.geometry.MultiLineString,
                    )
                    geoms = [
                        geom
                        for geom in intersection.geoms
                        if isinstance(geom, line_geoms)
                    ]
                intersection = shapely.ops.unary_union(geoms)
            orientation = orientationFor(self, other, triedReversed)
            return regionFromShapelyObject(intersection, orientation=orientation)
        return super().intersect(other, triedReversed)

    def union(self, other, triedReversed=False, buf=0):
        # If one of the regions isn't fixed, fall back on default behavior
        if isLazy(self) or isLazy(other):
            return super().union(other)

        if isinstance(other, PolygonalRegion):
            if self.z != other.z:
                return super().union(other, triedReversed)

        poly = toPolygon(other)

        if not poly:
            return super().union(other, triedReversed)
        union = polygonUnion((self.polygons, poly), buf=buf)
        orientation = VectorField.forUnionOf((self, other), tolerance=buf)
        return PolygonalRegion(polygon=union, orientation=orientation)

    def difference(self, other):
        # If one of the regions isn't fixed, fall back on default behavior
        if isLazy(self) or isLazy(other):
            return super().difference(other)

        if isinstance(other, PolygonalRegion):
            if self.z != other.z:
                return self

        poly = toPolygon(other)
        if poly is not None:
            diff = self.polygons - poly
            return regionFromShapelyObject(diff, orientation=self.orientation)
        return super().difference(other)

    @staticmethod
    @distributionFunction
    def unionAll(regions, buf=0):
        regions = tuple(regions)
        z = None
        for reg in regions:
            if z is not None and isinstance(reg, PolygonalRegion) and reg.z != z:
                raise ValueError(
                    "union of PolygonalRegions with different z values is undefined."
                )
            if isinstance(reg, PolygonalRegion) and z is None:
                z = reg.z

        regs, polys = [], []
        for reg in regions:
            if reg != nowhere:
                regs.append(reg)
                polys.append(toPolygon(reg))
        if not polys:
            return nowhere
        if any(not poly for poly in polys):
            raise TypeError(f"cannot take union of regions {regions}")
        union = polygonUnion(polys, buf=buf)
        orientation = VectorField.forUnionOf(regs, tolerance=buf)
        z = 0 if z is None else z
        return PolygonalRegion(polygon=union, orientation=orientation, z=z)

    @property
    @distributionFunction
    def boundary(self) -> "PolylineRegion":
        """Get the boundary of this region as a `PolylineRegion`."""
        return PolylineRegion(polyline=self.polygons.boundary)

    @distributionFunction
    def containsRegionInner(self, other, tolerance):
        poly = toPolygon(other)
        if poly is None:
            raise TypeError(f"cannot test inclusion of {other} in PolygonalRegion")
        return self.polygons.buffer(tolerance).contains(poly)

    @distributionFunction
    def distanceTo(self, point):
        point = toVector(point)
        dist2D = shapely.distance(self.polygons, makeShapelyPoint(point))
        return math.hypot(dist2D, point[2] - self.z)

    def projectVector(self, point, onDirection):
        raise NotImplementedError(
            f'{type(self).__name__} does not yet support projection using "on"'
        )

    @property
    def AABB(self):
        xmin, ymin, xmax, ymax = self.polygons.bounds
        return ((xmin, ymin), (xmax, ymax), (self.z, self.z))

    @distributionFunction
    def buffer(self, amount):
        buffered_polygons = self.polygons.buffer(amount)

        return PolygonalRegion(polygon=buffered_polygons, name=self.name, z=self.z)

    @property
    def dimensionality(self):
        return 2

    @cached_property
    def size(self):
        return self.polygons.area

    @property
    def area(self):
        return self.size

    def show(self, plt, style="r-", **kwargs):
        plotPolygon(self.polygons, plt, style=style, **kwargs)

    def __repr__(self):
        return f"PolygonalRegion({self.polygons!r}, {self.z!r})"

    def __eq__(self, other):
        if type(other) is not PolygonalRegion:
            return NotImplemented
        return (
            other.polygons == self.polygons
            and self.z == other.z
            and other.orientation == self.orientation
        )

    @cached
    def __hash__(self):
        return hash((self.polygons, self.orientation, self.z))


class CircularRegion(PolygonalRegion):
    """A circular region with a possibly-random center and radius.

    Args:
        center (`Vector`): center of the disc.
        radius (float): radius of the disc.
        resolution (int; optional): number of vertices to use when approximating this region as a
            polygon.
        name (str; optional): name for debugging.
    """

    def __init__(self, center, radius, resolution=32, name=None):
        self.center = toVector(center, "center of CircularRegion not a vector")
        self.radius = toScalar(radius, "radius of CircularRegion not a scalar")
        self.resolution = resolution
        self.circumcircle = (self.center, self.radius)

        deps = [self.center, self.radius]

        super().__init__(
            polygon=self._makePolygons(center, radius, resolution),
            z=self.center.z,
            name=name,
            additionalDeps=deps,
        )

    @staticmethod
    @distributionFunction
    def _makePolygons(center, radius, resolution):
        ctr = makeShapelyPoint(center)
        return ctr.buffer(radius, resolution=resolution)

    ## Lazy Construction Methods ##
    def sampleGiven(self, value):
        return CircularRegion(
            value[self.center],
            value[self.radius],
            name=self.name,
            resolution=self.resolution,
        )

    def evaluateInner(self, context):
        center = valueInContext(self.center, context)
        radius = valueInContext(self.radius, context)
        return CircularRegion(center, radius, name=self.name, resolution=self.resolution)

    def intersects(self, other, triedReversed=False):
        if isinstance(other, CircularRegion):
            return self.center.distanceTo(other.center) <= self.radius + other.radius

        return super().intersects(other, triedReversed)

    def containsPoint(self, point):
        point = toVector(point)

        if point.z != self.z:
            return False

        return point.distanceTo(self.center) <= self.radius

    def distanceTo(self, point):
        point = toVector(point)

        if point.z == 0:
            return max(0, point.distanceTo(self.center) - self.radius)

        return super().distanceTo(point)

    def uniformPointInner(self):
        x, y, z = self.center
        r = random.triangular(0, self.radius, self.radius)
        t = random.uniform(-math.pi, math.pi)
        pt = Vector(x + (r * cos(t)), y + (r * sin(t)), z)
        return self.orient(pt)

    @property
    def AABB(self):
        x, y, _ = self.center
        r = self.radius
        return ((x - r, y - r), (x + r, y + r), (self.z, self.z))

    def __repr__(self):
        return f"CircularRegion({self.center!r}, {self.radius!r})"


class SectorRegion(PolygonalRegion):
    """A sector of a `CircularRegion`.

    This region consists of a sector of a disc, i.e. the part of a disc subtended by a
    given arc.

    Args:
        center (`Vector`): center of the corresponding disc.
        radius (float): radius of the disc.
        heading (float): heading of the centerline of the sector.
        angle (float): angle subtended by the sector.
        resolution (int; optional): number of vertices to use when approximating this region as a
            polygon.
        name (str; optional): name for debugging.
    """

    def __init__(self, center, radius, heading, angle, resolution=32, name=None):
        self.center = toVector(center, "center of SectorRegion not a vector")
        self.radius = toScalar(radius, "radius of SectorRegion not a scalar")
        self.heading = toScalar(heading, "heading of SectorRegion not a scalar")
        self.angle = toScalar(angle, "angle of SectorRegion not a scalar")

        deps = [self.center, self.radius, self.heading, self.angle]

        r = (radius / 2) * cos(angle / 2)
        self.circumcircle = (self.center.offsetRadially(r, heading), r)
        self.resolution = resolution

        super().__init__(
            polygon=self._makePolygons(center, radius, heading, angle, resolution),
            z=self.center.z,
            name=name,
            additionalDeps=deps,
        )

    @staticmethod
    @distributionFunction
    def _makePolygons(center, radius, heading, angle, resolution):
        ctr = makeShapelyPoint(center)
        circle = ctr.buffer(radius, resolution=resolution)
        if angle >= math.tau - 0.001:
            polygon = circle
        else:
            half_angle = angle / 2
            mask = shapely.geometry.Polygon(
                [
                    center,
                    center.offsetRadially(radius, heading + half_angle),
                    center.offsetRadially(2 * radius, heading),
                    center.offsetRadially(radius, heading - half_angle),
                ]
            )
            polygon = circle & mask

        return polygon

    ## Lazy Construction Methods ##
    def sampleGiven(self, value):
        return SectorRegion(
            value[self.center],
            value[self.radius],
            value[self.heading],
            value[self.angle],
            name=self.name,
            resolution=self.resolution,
        )

    def evaluateInner(self, context):
        center = valueInContext(self.center, context)
        radius = valueInContext(self.radius, context)
        heading = valueInContext(self.heading, context)
        angle = valueInContext(self.angle, context)
        return SectorRegion(
            center, radius, heading, angle, name=self.name, resolution=self.resolution
        )

    @distributionFunction
    def containsPoint(self, point):
        point = toVector(point)

        if point.z != self.z:
            return False

        if not pointIsInCone(tuple(point), tuple(self.center), self.heading, self.angle):
            return False

        return point.distanceTo(self.center) <= self.radius

    def uniformPointInner(self):
        x, y, z = self.center
        heading, angle, maxDist = self.heading, self.angle, self.radius
        r = random.triangular(0, maxDist, maxDist)
        ha = angle / 2.0
        t = random.uniform(-ha, ha) + (heading + (math.pi / 2))
        pt = Vector(x + (r * cos(t)), y + (r * sin(t)), z)
        return self.orient(pt)

    def __repr__(self):
        return f"SectorRegion({self.center!r},{self.radius!r},{self.heading!r},{self.angle!r})"


class RectangularRegion(PolygonalRegion):
    """A rectangular region with a possibly-random position, heading, and size.

    Args:
        position (`Vector`): center of the rectangle.
        heading (float): the heading of the ``length`` axis of the rectangle.
        width (float): width of the rectangle.
        length (float): length of the rectangle.
        name (str; optional): name for debugging.
    """

    def __init__(self, position, heading, width, length, name=None):
        self.position = toVector(position, "position of RectangularRegion not a vector")
        self.heading = toScalar(heading, "heading of RectangularRegion not a scalar")
        self.width = toScalar(width, "width of RectangularRegion not a scalar")
        self.length = toScalar(length, "length of RectangularRegion not a scalar")

        deps = [self.position, self.heading, self.width, self.length]

        self.hw = hw = width / 2
        self.hl = hl = length / 2
        self.radius = hypot(hw, hl)  # circumcircle; for collision detection
        self.corners = tuple(
            self.position.offsetRotated(heading, Vector(*offset))
            for offset in ((hw, hl), (-hw, hl), (-hw, -hl), (hw, -hl))
        )
        self.circumcircle = (self.position, self.radius)

        super().__init__(
            polygon=self._makePolygons(position, heading, width, length),
            z=self.position.z,
            name=name,
            additionalDeps=deps,
        )

    @staticmethod
    @distributionFunction
    def _makePolygons(position, heading, width, length):
        unitBox = shapely.geometry.Polygon(
            ((0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5))
        )
        cyaw, syaw = math.cos(heading), math.sin(heading)
        matrix = [
            width * cyaw,
            -length * syaw,
            width * syaw,
            length * cyaw,
            position[0],
            position[1],
        ]
        return shapely.affinity.affine_transform(unitBox, matrix)

    ## Lazy Construction Methods ##
    def sampleGiven(self, value):
        return RectangularRegion(
            value[self.position],
            value[self.heading],
            value[self.width],
            value[self.length],
            name=self.name,
        )

    def evaluateInner(self, context):
        position = valueInContext(self.position, context)
        heading = valueInContext(self.heading, context)
        width = valueInContext(self.width, context)
        length = valueInContext(self.length, context)
        return RectangularRegion(position, heading, width, length, name=self.name)

    def uniformPointInner(self):
        hw, hl = self.hw, self.hl
        rx = random.uniform(-hw, hw)
        ry = random.uniform(-hl, hl)
        pt = self.position.offsetRotated(self.heading, Vector(rx, ry, 0))
        return self.orient(pt)

    @property
    def AABB(self):
        x, y, z = zip(*self.corners)
        minx, maxx = findMinMax(x)
        miny, maxy = findMinMax(y)
        return ((minx, miny), (maxx, maxy), (self.z, self.z))

    def __repr__(self):
        return (
            f"RectangularRegion({self.position!r}, {self.heading!r}, "
            f"{self.width!r}, {self.length!r})"
        )


class PolylineRegion(Region):
    """Region given by one or more polylines (chain of line segments).

    The region may be specified by giving either a sequence of points or ``shapely``
    polylines (a ``LineString`` or ``MultiLineString``).

    Args:
        points: sequence of points making up the polyline (or `None` if using the
            **polyline** argument instead).
        polyline: ``shapely`` polyline or collection of polylines (or `None` if using
            the **points** argument instead).
        orientation (optional): :term:`preferred orientation` to use, or `True` to use an
            orientation aligned with the direction of the polyline (the default).
        name (str; optional): name for debugging.
    """

    def __init__(self, points=None, polyline=None, orientation=True, name=None):
        if orientation is True:
            orientation = VectorField("Polyline", self.defaultOrientation)
            self.usingDefaultOrientation = True
        else:
            self.usingDefaultOrientation = False
        super().__init__(name, orientation=orientation)

        if points is not None:
            points = tuple(points)

            if any(len(point) > 2 and point[2] != 0 for point in points):
                warnings.warn(
                    '"points" passed to PolylineRegion have nonzero Z'
                    " components. These will be replaced with 0."
                )

            points = tuple((p[0], p[1], 0) for p in points)

            if len(points) < 2:
                raise ValueError("tried to create PolylineRegion with < 2 points")
            self.points = points
            self.lineString = shapely.geometry.LineString(points)
        elif polyline is not None:
            if isinstance(polyline, shapely.geometry.LineString):
                if len(polyline.coords) < 2:
                    raise ValueError(
                        "tried to create PolylineRegion with <2-point LineString"
                    )
            elif isinstance(polyline, shapely.geometry.MultiLineString):
                if len(polyline.geoms) == 0:
                    raise ValueError(
                        "tried to create PolylineRegion from empty MultiLineString"
                    )
                for line in polyline.geoms:
                    assert len(line.coords) >= 2
            else:
                raise ValueError("tried to create PolylineRegion from non-LineString")
            self.lineString = polyline
            self.points = None
        else:
            raise ValueError("must specify points or polyline for PolylineRegion")

        if not self.lineString.is_valid:
            raise ValueError(
                "tried to create PolylineRegion with "
                f"invalid LineString {self.lineString}"
            )
        shapely.prepare(self.lineString)
        self.segments = self.segmentsOf(self.lineString)
        cumulativeLengths = []
        total = 0
        for p, q in self.segments:
            dx, dy = p[0] - q[0], p[1] - q[1]
            total += math.hypot(dx, dy)
            cumulativeLengths.append(total)
        self.cumulativeLengths = cumulativeLengths
        if self.points is None:
            pts = []
            last = None
            for p, q in self.segments:
                if last is None or p[:2] != last[:2]:
                    pts.append(toVector(p).coordinates)
                pts.append(toVector(q).coordinates)
                last = q
            self.points = tuple(pts)

    @classmethod
    def segmentsOf(cls, lineString):
        if isinstance(lineString, shapely.geometry.LineString):
            segments = []
            points = list(lineString.coords)
            if len(points) < 2:
                raise ValueError("LineString has fewer than 2 points")
            last = points[0][:2]
            for point in points[1:]:
                segments.append((last, point))
                last = point
            return segments
        elif isinstance(lineString, shapely.geometry.MultiLineString):
            allSegments = []
            for line in lineString.geoms:
                allSegments.extend(cls.segmentsOf(line))
            return allSegments
        else:
            raise ValueError("called segmentsOf on non-linestring")

    @cached_property
    def start(self):
        """Get an `OrientedPoint` at the start of the polyline.

        The OP's orientation will be aligned with the orientation of the region, if
        there is one (the default orientation pointing along the polyline).
        """
        pointA, pointB = self.segments[0]
        if self.usingDefaultOrientation:
            orientation = headingOfSegment(pointA, pointB)
        elif self.orientation is not None:
            orientation = self.orientation[Vector(*pointA)]
        else:
            orientation = 0
        orientation = toOrientation(orientation)
        from scenic.core.object_types import OrientedPoint

        return OrientedPoint._with(
            position=pointA,
            yaw=orientation.yaw,
            pitch=orientation.pitch,
            roll=orientation.roll,
        )

    @cached_property
    def end(self):
        """Get an `OrientedPoint` at the end of the polyline.

        The OP's orientation will be aligned with the orientation of the region, if
        there is one (the default orientation pointing along the polyline).
        """
        pointA, pointB = self.segments[-1]
        if self.usingDefaultOrientation:
            orientation = headingOfSegment(pointA, pointB)
        elif self.orientation is not None:
            orientation = self.orientation[Vector(*pointB)].yaw
        else:
            orientation = 0
        from scenic.core.object_types import OrientedPoint

        orientation = toOrientation(orientation)
        from scenic.core.object_types import OrientedPoint

        return OrientedPoint._with(
            position=pointB,
            yaw=orientation.yaw,
            pitch=orientation.pitch,
            roll=orientation.roll,
        )

    def defaultOrientation(self, point):
        start, end = self.nearestSegmentTo(point)
        return start.angleTo(end)

    def uniformPointInner(self):
        pointA, pointB = random.choices(
            self.segments, cum_weights=self.cumulativeLengths
        )[0]
        interpolation = random.random()
        x, y = averageVectors(pointA, pointB, weight=interpolation)
        if self.usingDefaultOrientation:
            return OrientedVector(x, y, 0, headingOfSegment(pointA, pointB))
        else:
            return self.orient(Vector(x, y, 0))

    def containsRegionInner(self, other, tolerance):
        poly = toPolygon(other)
        if poly is None:
            raise TypeError(f"cannot test inclusion of {other} in PolylineRegion")
        return self.polygons.buffer(tolerance).contains(poly)

    def intersect(self, other, triedReversed=False):
        poly = toPolygon(other)
        if poly is not None:
            if isinstance(other, PolygonalRegion) and other.z != 0:
                return super().intersect(other, triedReversed)
            intersection = self.lineString & poly
            line_geoms = (shapely.geometry.LineString, shapely.geometry.MultiLineString)
            if (
                isinstance(intersection, shapely.geometry.GeometryCollection)
                and intersection.length > 0
            ):
                geoms = [
                    geom for geom in intersection.geoms if isinstance(geom, line_geoms)
                ]
                intersection = shapely.ops.unary_union(geoms)
            orientation = orientationFor(self, other, triedReversed)
            return regionFromShapelyObject(intersection, orientation=orientation)
        return super().intersect(other, triedReversed)

    def intersects(self, other, triedReversed=False):
        poly = toPolygon(other)
        if poly is not None:
            return self.lineString.intersects(poly)
        return super().intersects(other, triedReversed)

    def difference(self, other):
        poly = toPolygon(other)
        if poly is not None:
            diff = self.lineString - poly
            return regionFromShapelyObject(diff)
        return super().difference(other)

    @staticmethod
    def unionAll(regions):
        regions = tuple(regions)
        if not regions:
            return nowhere
        if any(not isinstance(region, PolylineRegion) for region in regions):
            raise TypeError(f"cannot take Polyline union of regions {regions}")
        # take union by collecting LineStrings, to preserve the order of points
        strings = []
        for region in regions:
            string = region.lineString
            if isinstance(string, shapely.geometry.MultiLineString):
                strings.extend(string.geoms)
            else:
                strings.append(string)
        newString = shapely.geometry.MultiLineString(strings)
        return PolylineRegion(polyline=newString)

    @distributionMethod
    def containsPoint(self, point) -> bool:
        point = toVector(point)
        if point.z != 0:
            return False
        return shapely.intersects_xy(self.lineString, point.x, point.y)

    def containsObject(self, obj):
        return False

    @distributionMethod
    def distanceTo(self, point) -> float:
        point = toVector(point)
        dist2D = self.lineString.distance(makeShapelyPoint(point))
        return math.hypot(dist2D, point.z)

    def projectVector(self, point, onDirection):
        raise TypeError('PolylineRegion does not support projection using "on"')

    @distributionMethod
    def signedDistanceTo(self, point) -> float:
        """Compute the signed distance of the PolylineRegion to a point.

        The distance is positive if the point is left of the nearest segment,
        and negative otherwise.
        """
        dist = self.distanceTo(point)
        start, end = self.nearestSegmentTo(point)
        rp = point - start
        tangent = end - start
        return dist if tangent.angleWith(rp) >= 0 else -dist

    @distributionMethod
    def project(self, point):
        pt = shapely.ops.nearest_points(self.lineString, makeShapelyPoint(point))[0]
        return Vector(*pt.coords[0])

    @distributionMethod
    def nearestSegmentTo(self, point):
        dist = self.lineString.project(makeShapelyPoint(point))
        # TODO optimize?
        for segment, cumLen in zip(self.segments, self.cumulativeLengths):
            if dist <= cumLen:
                break
        # FYI, could also get here if loop runs to completion due to rounding error
        return (Vector(*segment[0]), Vector(*segment[1]))

    def pointAlongBy(self, distance, normalized=False) -> Vector:
        """Find the point a given distance along the polyline from its start.

        If **normalized** is true, then distance should be between 0 and 1, and
        is interpreted as a fraction of the length of the polyline. So for example
        ``pointAlongBy(0.5, normalized=True)`` returns the polyline's midpoint.
        """
        pt = self.lineString.interpolate(distance, normalized=normalized)
        return Vector(pt.x, pt.y)

    def equallySpacedPoints(self, num):
        return [self.pointAlongBy(d) for d in numpy.linspace(0, self.length, num)]

    def pointsSeparatedBy(self, distance):
        return [self.pointAlongBy(d) for d in numpy.arange(0, self.length, distance)]

    @property
    def dimensionality(self):
        return 1

    @cached_property
    def size(self):
        return self.lineString.length

    @property
    def length(self):
        return self.size

    @property
    def AABB(self):
        xmin, ymin, xmax, ymax = self.lineString.bounds
        return ((xmin, ymin), (xmax, ymax), (0, 0))

    def show(self, plt, style="r-", **kwargs):
        plotPolygon(self.lineString, plt, style=style, **kwargs)

    def __getitem__(self, i) -> Vector:
        """Get the ith point along this polyline.

        If the region consists of multiple polylines, this order is linear
        along each polyline but arbitrary across different polylines.
        """
        return Vector(*self.points[i])

    def __add__(self, other):
        if not isinstance(other, PolylineRegion):
            return NotImplemented
        # take union by collecting LineStrings, to preserve the order of points
        strings = []
        for region in (self, other):
            string = region.lineString
            if isinstance(string, shapely.geometry.MultiLineString):
                strings.extend(string.geoms)
            else:
                strings.append(string)
        newString = shapely.geometry.MultiLineString(strings)
        return PolylineRegion(polyline=newString)

    def __len__(self) -> int:
        """Get the number of vertices of the polyline."""
        return len(self.points)

    def __repr__(self):
        return f"PolylineRegion({self.lineString!r})"

    def __eq__(self, other):
        if type(other) is not PolylineRegion:
            return NotImplemented
        return other.lineString == self.lineString

    @cached
    def __hash__(self):
        return hash(self.lineString)


class PointSetRegion(Region):
    """Region consisting of a set of discrete points.

    No `Object` can be contained in a `PointSetRegion`, since the latter is discrete.
    (This may not be true for subclasses, e.g. `GridRegion`.)

    Args:
        name (str): name for debugging
        points (arraylike): set of points comprising the region
        kdTree (`scipy.spatial.KDTree`, optional): k-D tree for the points (one will
            be computed if none is provided)
        orientation (`VectorField`; optional): :term:`preferred orientation` for the
            region
        tolerance (float; optional): distance tolerance for checking whether a point lies
            in the region
    """

    def __init__(self, name, points, kdTree=None, orientation=None, tolerance=1e-6):
        super().__init__(name, orientation=orientation)

        if kdTree is not None:
            warnings.warn(
                "Passing a kdTree to the PointSetRegion is deprecated."
                "The value will be ignored and the parameter removed in future versions.",
                DeprecationWarning,
            )

        for point in points:
            if any(isLazy(coord) for coord in point):
                raise ValueError("only fixed PointSetRegions are supported")

        # Try to convert immediately, but catch case which has 2D and 3D points
        try:
            self.points = numpy.asarray(points)
        except ValueError:
            self.points = numpy.asarray([Vector(*pt) for pt in points])

        if self.points.shape[1] == 2:
            self.points = numpy.hstack((self.points, numpy.zeros((len(self.points), 1))))
        elif self.points.shape[1] == 3:
            pass
        else:
            raise ValueError(f"The points parameter had incorrect shape {points.shape}")

        assert self.points.shape[1] == 3

        import scipy.spatial  # slow import not often needed

        self.kdTree = scipy.spatial.KDTree(self.points)
        self.orientation = orientation
        self.tolerance = tolerance

    def uniformPointInner(self):
        i = random.randrange(0, len(self.points))
        return self.orient(Vector(*self.points[i]))

    def intersects(self, other, triedReversed=False):
        return any(other.containsPoint(pt) for pt in self.points)

    def intersect(self, other, triedReversed=False):
        def sampler(intRegion):
            o = intRegion.regions[1]
            center, radius = o.circumcircle
            possibles = (
                Vector(*self.kdTree.data[i])
                for i in self.kdTree.query_ball_point(center, radius)
            )
            intersection = [p for p in possibles if o.containsPoint(p)]
            if len(intersection) == 0:
                raise RejectionException(f"empty intersection of Regions {self} and {o}")
            return self.orient(random.choice(intersection))

        orientation = orientationFor(self, other, triedReversed)
        return IntersectionRegion(self, other, sampler=sampler, orientation=orientation)

    def containsPoint(self, point):
        point = toVector(point).coordinates
        distance, location = self.kdTree.query(point)
        return distance <= self.tolerance

    def containsObject(self, obj):
        return False

    def containsRegionInner(self, reg, tolerance):
        if isinstance(reg, PointSetRegion):
            return all(
                distance <= tolerance for distance, _ in self.kdTree.query(reg.points)
            )

        raise NotImplementedError

    @property
    def dimensionality(self):
        return 0

    @cached_property
    def size(self):
        return len(self.points)

    @distributionMethod
    def distanceTo(self, point):
        point = toVector(point).coordinates
        distance, _ = self.kdTree.query(point)
        return distance

    def projectVector(self, point, onDirection):
        raise TypeError('PointSetRegion does not support projection using "on"')

    @property
    def AABB(self):
        return tuple(
            zip(numpy.amin(self.points, axis=0), numpy.amax(self.points, axis=0))
        )

    def __eq__(self, other):
        if type(other) is not PointSetRegion:
            return NotImplemented
        return (
            self.name == other.name
            and numpy.array_equal(self.points, other.points)
            and self.orientation == other.orientation
        )

    @cached
    def __hash__(self):
        return hash((self.name, self.points.tobytes(), self.orientation))


@distributionFunction
def convertToFootprint(region):
    """Recursively convert a region into it's footprint.

    For a polygonal region, returns the footprint. For composed regions,
    recursively reconstructs them using the footprints of their sub regions.
    """
    if isinstance(region, PolygonalRegion):
        return region.footprint

    if isinstance(region, IntersectionRegion):
        subregions = [convertToFootprint(r) for r in region.regions]
        return IntersectionRegion(*subregions)

    if isinstance(region, DifferenceRegion):
        return DifferenceRegion(
            convertToFootprint(region.regionA), convertToFootprint(region.regionB)
        )

    return region


###################################################################################################
# Niche Regions
###################################################################################################


class GridRegion(PointSetRegion):
    """A Region given by an obstacle grid.

    A point is considered to be in a `GridRegion` if the nearest grid point is
    not an obstacle.

    Args:
        name (str): name for debugging
        grid: 2D list, tuple, or NumPy array of 0s and 1s, where 1 indicates an obstacle
          and 0 indicates free space
        Ax (float): spacing between grid points along X axis
        Ay (float): spacing between grid points along Y axis
        Bx (float): X coordinate of leftmost grid column
        By (float): Y coordinate of lowest grid row
        orientation (`VectorField`; optional): orientation of region
    """

    def __init__(self, name, grid, Ax, Ay, Bx, By, orientation=None):
        self.grid = numpy.array(grid)
        self.sizeY, self.sizeX = self.grid.shape
        self.Ax, self.Ay = Ax, Ay
        self.Bx, self.By = Bx, By
        y, x = numpy.where(self.grid == 0)
        points = [self.gridToPoint(point) for point in zip(x, y)]
        super().__init__(name, points, orientation=orientation)

    def gridToPoint(self, gp):
        x, y = gp
        return ((self.Ax * x) + self.Bx, (self.Ay * y) + self.By)

    def pointToGrid(self, point):
        x, y, z = point
        x = (x - self.Bx) / self.Ax
        y = (y - self.By) / self.Ay
        nx = int(round(x))
        if nx < 0 or nx >= self.sizeX:
            return None
        ny = int(round(y))
        if ny < 0 or ny >= self.sizeY:
            return None
        return (nx, ny)

    def containsPoint(self, point):
        gp = self.pointToGrid(point)
        if gp is None:
            return False
        x, y = gp
        return self.grid[y, x] == 0

    def containsObject(self, obj):
        # TODO improve this procedure!
        # Fast check
        for c in obj._corners2D:
            if not self.containsPoint(c):
                return False
        # Slow check
        gps = [self.pointToGrid(corner) for corner in obj._corners2D]
        x, y = zip(*gps)
        minx, maxx = findMinMax(x)
        miny, maxy = findMinMax(y)
        for x in range(minx, maxx + 1):
            for y in range(miny, maxy + 1):
                p = self.gridToPoint((x, y))
                if self.grid[y, x] == 1 and obj.containsPoint(p):
                    return False
        return True

    def containsRegionInner(self, reg, tolerance):
        raise NotImplementedError

    def projectVector(self, point, onDirection):
        raise TypeError('GridRegion does not support projection using "on"')


###################################################################################################
# View Regions
###################################################################################################


class ViewRegion(MeshVolumeRegion):
    """The viewing volume of a camera defined by a radius and horizontal/vertical view angles.

    The default view region can take several forms, depending on the viewAngles parameter:

    * Case 1:       viewAngles[1] = 180 degrees

      * Case 2.a    viewAngles[0] = 360 degrees     => Sphere
      * Case 2.b    viewAngles[0] < 360 degrees     => Sphere & CylinderSectionRegion

    * Case 2:       viewAngles[1] < 180 degrees

      * Case 2.a    viewAngles[0] = 360 degrees     => Sphere - (Cone + Cone) (Cones on z axis expanding from origin)
      * Case 2.b    viewAngles[0] < 360 degrees     => Sphere & ViewSectionRegion

    When making changes to this class you should run ``pytest -k test_viewRegion --exhaustive``.

    Args:
        visibleDistance: The view distance for this region.
        viewAngles: The view angles for this region.
        name: An optional name to help with debugging.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        angleCutoff: How close to 180/360 degrees an angle has to be to be mapped to that value.
        tolerance: Tolerance for collision computations.
    """

    def __init__(
        self,
        visibleDistance,
        viewAngles,
        name=None,
        position=Vector(0, 0, 0),
        rotation=None,
        orientation=None,
        angleCutoff=0.01,
        tolerance=1e-8,
    ):
        # Bound viewAngles from either side.
        if min(viewAngles) <= 0:
            raise ValueError("viewAngles cannot have a component less than or equal to 0")

        view_region = None
        diameter = 2 * visibleDistance
        base_sphere = SpheroidRegion(
            dimensions=(diameter, diameter, diameter), engine="scad"
        )

        if math.pi - angleCutoff <= viewAngles[1]:
            # Case 1
            if math.tau - angleCutoff <= viewAngles[0]:
                # Case 1.a
                view_region = base_sphere
            else:
                view_region = base_sphere.intersect(
                    CylinderSectionRegion(visibleDistance, viewAngles[0])
                )
        else:
            # Case 2
            if math.tau - angleCutoff <= viewAngles[0]:
                # Case 2.a
                # Create cone with yaw oriented around (0,0,-1)
                padded_height = visibleDistance * 2
                radius = padded_height * math.tan((math.pi - viewAngles[1]) / 2)

                cone_mesh = trimesh.creation.cone(radius=radius, height=padded_height)

                position_matrix = translation_matrix((0, 0, -1 * padded_height))
                cone_mesh.apply_transform(position_matrix)

                # Create two cones around the yaw axis
                orientation_1 = Orientation._fromEuler(0, 0, 0)
                orientation_2 = Orientation._fromEuler(0, 0, math.pi)

                cone_1 = MeshVolumeRegion(
                    mesh=cone_mesh, rotation=orientation_1, centerMesh=False
                )
                cone_2 = MeshVolumeRegion(
                    mesh=cone_mesh, rotation=orientation_2, centerMesh=False
                )

                view_region = base_sphere.difference(cone_1).difference(cone_2)
            else:
                # Case 2.b
                view_region = base_sphere.intersect(
                    ViewSectionRegion(visibleDistance, viewAngles)
                )

        assert view_region is not None

        # Initialize volume region
        super().__init__(
            mesh=view_region.mesh,
            name=name,
            position=position,
            rotation=rotation,
            orientation=orientation,
            tolerance=tolerance,
            centerMesh=False,
        )


class ViewSectionRegion(MeshVolumeRegion):
    def __init__(self, visibleDistance, viewAngles, rotation=None, resolution=32):
        triangles = []

        # Face line
        top_line = []
        bot_line = []

        for azimuth in numpy.linspace(
            -viewAngles[0] / 2, viewAngles[0] / 2, num=resolution
        ):
            top_line.append(
                self.vecFromAziAlt(azimuth, viewAngles[1] / 2, 2 * visibleDistance)
            )
            bot_line.append(
                self.vecFromAziAlt(azimuth, -viewAngles[1] / 2, 2 * visibleDistance)
            )

        # Face triangles
        for li in range(len(top_line) - 1):
            triangles.append((top_line[li], bot_line[li], top_line[li + 1]))
            triangles.append((bot_line[li], bot_line[li + 1], top_line[li + 1]))

        # Side triangles
        triangles.append((bot_line[0], top_line[0], (0, 0, 0)))
        triangles.append((top_line[-1], bot_line[-1], (0, 0, 0)))

        # Top/Bottom triangles
        for li in range(len(top_line) - 1):
            triangles.append((top_line[li], top_line[li + 1], (0, 0, 0)))
            triangles.append((bot_line[li + 1], bot_line[li], (0, 0, 0)))

        vr_mesh = trimesh.Trimesh(**trimesh.triangles.to_kwargs(triangles))

        assert vr_mesh.is_volume

        super().__init__(mesh=vr_mesh, rotation=rotation, centerMesh=False)

    @staticmethod
    def vecFromAziAlt(azimuth, altitude, flat_dist):
        return flat_dist * numpy.asarray(
            [-math.sin(azimuth), math.cos(azimuth), math.tan(altitude)]
        )


class CylinderSectionRegion(MeshVolumeRegion):
    def __init__(self, visibleDistance, viewAngle, rotation=None, resolution=32):
        triangles = []

        # Face line
        top_line = []
        bot_line = []

        for azimuth in numpy.linspace(-viewAngle / 2, viewAngle / 2, num=resolution):
            top_line.append(
                self.vecFromAzi(azimuth, 2 * visibleDistance, 2 * visibleDistance)
            )
            bot_line.append(
                self.vecFromAzi(azimuth, -2 * visibleDistance, 2 * visibleDistance)
            )

        # Face triangles
        for li in range(len(top_line) - 1):
            triangles.append((top_line[li], bot_line[li], top_line[li + 1]))
            triangles.append((bot_line[li], bot_line[li + 1], top_line[li + 1]))

        # Side triangles
        triangles.append((bot_line[0], top_line[0], (0, 0, 2 * visibleDistance)))
        triangles.append(
            (bot_line[0], (0, 0, 2 * visibleDistance), (0, 0, -2 * visibleDistance))
        )
        triangles.append((top_line[-1], bot_line[-1], (0, 0, 2 * visibleDistance)))
        triangles.append(
            ((0, 0, 2 * visibleDistance), bot_line[-1], (0, 0, -2 * visibleDistance))
        )

        # Top/Bottom triangles
        for li in range(len(top_line) - 1):
            triangles.append(
                (top_line[li], top_line[li + 1], (0, 0, 2 * visibleDistance))
            )
            triangles.append(
                (bot_line[li + 1], bot_line[li], (0, 0, -2 * visibleDistance))
            )

        vr_mesh = trimesh.Trimesh(**trimesh.triangles.to_kwargs(triangles))

        assert vr_mesh.is_volume

        super().__init__(mesh=vr_mesh, rotation=rotation, centerMesh=False)

    @staticmethod
    def vecFromAzi(azimuth, height, dist):
        raw_vec = dist * numpy.asarray([-math.sin(azimuth), math.cos(azimuth), 0])
        raw_vec[2] = height
        return raw_vec
