"""Objects representing regions in space.

Manipulations of polygons and line segments are done using the
`shapely <https://github.com/shapely/shapely>`_ package.

Manipulations of meshes is done using the
`trimesh <https://trimsh.org/>`_ package.
"""

import math
import random
import itertools
from functools import lru_cache
from abc import ABC, abstractmethod
import warnings

import numpy
import scipy
import shapely
import shapely.geometry
from shapely.geometry import MultiPolygon
import shapely.ops
import shapely.prepared

import trimesh
from trimesh.transformations import translation_matrix, quaternion_matrix, concatenate_matrices

from subprocess import CalledProcessError

from scenic.core.distributions import (Samplable, RejectionException, needsSampling, needsLazyEvaluation,
                                       distributionFunction, distributionMethod, toDistribution)
from scenic.core.lazy_eval import valueInContext
from scenic.core.vectors import Vector, OrientedVector, VectorDistribution, VectorField, Orientation
from scenic.core.geometry import _RotatedRectangle
from scenic.core.geometry import sin, cos, hypot, findMinMax, pointIsInCone, averageVectors
from scenic.core.geometry import headingOfSegment, triangulatePolygon, plotPolygon, polygonUnion
from scenic.core.type_support import toVector, toScalar, toOrientation
from scenic.core.utils import cached, cached_property

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
    def intersects(self, other, triedReversed=False) -> bool:
        """intersects(other)

        Check if this `Region` intersects another.
        """
        if triedReversed:
            raise NotImplementedError
        else:
            return other.intersects(self, triedReversed=True)

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
        """Check if the `Region` contains an :obj:`~scenic.core.object_types.Object`.
        The default implementation assumes the `Region` is convex; subclasses must
        override the method if this is not the case.
        """
        pass

    def containsRegion(self, reg, tolerance=0):
        raise NotImplementedError

    @distributionMethod
    def distanceTo(self, point) -> float:
        """Distance to this region from a given point.

        Not supported by all region types.
        """
        raise NotImplementedError

    ## Overridable Methods ##
    # The following methods can be overriden to get better performance or if the region
    # has dependencies (in the case of sampleGiven).

    @property
    def dimensionality(self):
        return None

    @property
    def size(self):
        return None

    def intersect(self, other, triedReversed=False) -> 'Region':
        """intersect(other)

        Get a `Region` representing the intersection of this one with another.

        If both regions have a :term:`preferred orientation`, the one of ``self``
        is inherited by the intersection.
        """
        if triedReversed:
            orientation = orientationFor(self, other, triedReversed)
            return IntersectionRegion(self, other, orientation=orientation)
        else:
            return other.intersect(self, triedReversed=True)

    def union(self, other, triedReversed=False) -> 'Region':
        """Get a `Region` representing the union of this one with another.
        Not supported by all region types.
        """
        if triedReversed:
            raise NotImplementedError(f"Cannot compute union of {type(self)} and {type(other)}")
        else:
            return other.union(self, triedReversed=True)

    def difference(self, other) -> 'Region':
        """Get a `Region` representing the difference of this one and another."""
        if isinstance(other, EmptyRegion):
            return self
        elif isinstance(other, AllRegion):
            return nowhere
        return DifferenceRegion(self, other)

    def getAABB(self):
        """Axis-aligned bounding box for this `Region`. Implemented by some subclasses."""
        raise NotImplementedError

    def sampleGiven(self, value):
        return self

    ## API Methods ##

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
        s = f'<{type(self).__name__}'
        if self.name:
            s += f' {self.name}'
        return s + '>'

    def __repr__(self):
        s = f'<{type(self).__name__}'
        if self.name:
            s += f' {self.name}'
        return s + f' at {hex(id(self))}>'

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

    def __repr__(self):
        return f'PointIn({self.region!r})'

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
        raise RuntimeError(f'Attempted to sample from everywhere (AllRegion)')

    def containsPoint(self, point):
        return True

    def containsObject(self, obj):
        return True

    def containsRegion(self, reg, tolerance=0):
        return True

    def distanceTo(self, point):
        return 0

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
        raise RejectionException(f'sampling empty Region')

    def containsPoint(self, point):
        return False

    def containsObject(self, obj):
        return False

    def containsRegion(self, reg, tolerance=0):
        return type(reg) is EmptyRegion

    def distanceTo(self, point):
        return float('inf')

    def show(self, plt, style=None, **kwargs):
        pass

    def __eq__(self, other):
        return type(other) is EmptyRegion

    def __hash__(self):
        return hash(EmptyRegion)

#: A `Region` containing all points.
#:
#: Points may not be sampled from this region, as no uniform distribution over it exists.
everywhere = AllRegion('everywhere')

#: A `Region` containing no points.
#:
#: Attempting to sample from this region causes the sample to be rejected.
nowhere = EmptyRegion('nowhere')

class IntersectionRegion(Region):
    def __init__(self, *regions, orientation=None, sampler=None, name=None):
        self.regions = tuple(regions)
        if len(self.regions) < 2:
            raise ValueError('tried to take intersection of fewer than 2 regions')
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
        return IntersectionRegion(*regs, orientation=value[self.orientation],
                                  sampler=self.sampler, name=self.name)

    def evaluateInner(self, context):
        regs = (valueInContext(reg, context) for reg in self.regions)
        orientation = valueInContext(self.orientation, context)
        return IntersectionRegion(*regs, orientation=orientation, sampler=self.sampler,
                                  name=self.name)

    def containsPoint(self, point):
        return all(region.containsPoint(point) for region in self.footprint.regions)

    def containsObject(self, obj):
        return all(region.containsObject(obj) for region in self.footprint.regions)

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
            raise UndefinedSamplingException(f"All regions in {regs}"
                " do not support sampling, so the intersection doesn't either.")

        # Check each point for containment each region.
        for point in points:
            if all(region.containsPoint(point) for region in regs):
                return point

        raise RejectionException(f'sampling intersection of Regions {regs}')

    def isEquivalentTo(self, other):
        if type(other) is not IntersectionRegion:
            return False
        return (areEquivalent(set(other.regions), set(self.regions))
                and other.orientation == self.orientation)

    def __repr__(self):
        return f'IntersectionRegion({self.regions!r})'

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
        return DifferenceRegion(regionA, regionB, orientation=orientation,
                                sampler=self.sampler, name=self.name)

    def containsPoint(self, point):
        return self.footprint.regionA.containsPoint(point) and not self.footprint.regionB.containsPoint(point)

    def containsObject(self, obj):
        return self.footprint.regionA.containsObject(obj) and not self.footprint.regionB.intersects(obj.occupiedSpace)

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
                f'sampling difference of Regions {regionA} and {regionB}')
        return point

    def isEquivalentTo(self, other):
        if type(other) is not DifferenceRegion:
            return False
        return (areEquivalent(self.regionA, other.regionA)
                and areEquivalent(self.regionB, other.regionB)
                and other.orientation == self.orientation)

    def __repr__(self):
        return f'DifferenceRegion({self.regionA!r}, {self.regionB!r})'

def toPolygon(thing):
    if needsSampling(thing):
        return None
    if hasattr(thing, 'polygon'):
        poly = thing.polygon
    elif hasattr(thing, 'polygons'):
        poly = thing.polygons
    elif hasattr(thing, 'lineString'):
        poly = thing.lineString
    else:
        return None

    if poly.has_z:  # TODO revisit once we have 3D regions
        return shapely.ops.transform(lambda x, y, z: (x, y), poly)
    else:
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
        return PointSetRegion('PointSet', points, orientation=orientation)
    elif isinstance(obj, shapely.geometry.Point):
        return PointSetRegion('PointSet', obj.coords, orientation=orientation)
    else:
        raise TypeError(f'unhandled type of Shapely geometry: {obj}')

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
    """ A Trimesh object that always returns non-convex.

    Used so that fcl doesn't find collision without an actual surface
    intersection.
    """
    @property
    def is_convex(self):
        return False

class MeshRegion(Region):
    """Region given by a scaled, positioned, and rotated mesh. 

    This region can be subclassed to define whether operations are performed over the volume or surface of the mesh.

    The mesh is first placed so the origin is at the center of the bounding box (unless center_mesh is ``False``).
    The mesh is scaled to ``dimensions``, translated so the center of the bounding box of the mesh is at ``positon``,
    and then rotated to ``rotation``.

    Args:
        mesh: The base mesh for this MeshRegion.
        name: An optional name to help with debugging.
        dimensions: An optional 3-tuple, with the values representing width, length, height respectively.
          The mesh will be scaled such that the bounding box for the mesh has these dimensions.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for collision computations.
        center_mesh: Whether or not to center the mesh after copying and before transformations. Only turn this off
          if you know what you're doing and don't plan to scale or translate the mesh.
        engine: Which engine to use for mesh operations. Either "blender" or "scad".
        additional_deps: Any additional sampling dependencies this region relies on.
    """
    def __init__(self, mesh, name=None, dimensions=None, position=None, rotation=None, orientation=None,\
        tolerance=1e-6, center_mesh=True, engine="blender", additional_deps=[]):
        # Copy the mesh and parameters
        if needsSampling(mesh):
            self._mesh = mesh
        else:
            if isinstance(mesh, trimesh.primitives._Primitive):
                self._mesh = mesh.to_mesh()
            elif isinstance(mesh, trimesh.base.Trimesh):
                self._mesh = mesh.copy()
            else:
                raise ValueError(f"Got unexpected mesh parameter of type {type(mesh)}")

        self.dimensions = None if dimensions is None else toVector(dimensions)
        self.position = None if position is None else toVector(position)
        self.rotation = None if rotation is None else toOrientation(rotation)
        self.orientation = None if orientation is None else toDistribution(orientation)
        self.tolerance = tolerance
        self.center_mesh = center_mesh
        self.engine = engine

        # Initialize superclass with samplables
        super().__init__(name, self._mesh, self.dimensions, self.position, self.rotation, *additional_deps, orientation=orientation)

        # If sampling is needed, delay transformations
        if needsSampling(self) or needsLazyEvaluation(self):
            return

        # Center mesh unless disabled
        if center_mesh:
            self.mesh.vertices -= self.mesh.bounding_box.center_mass

        # If dimensions are provided, scale mesh to those dimension
        if self.dimensions is not None:
            scale = self.mesh.extents / numpy.array(self.dimensions)

            scale_matrix = numpy.eye(4)
            scale_matrix[:3, :3] /= scale

            self.mesh.apply_transform(scale_matrix)

        # If rotation is provided, apply rotation
        if self.rotation is not None:
            rotation_matrix = quaternion_matrix((self.rotation.w, self.rotation.x, self.rotation.y, self.rotation.z))
            self.mesh.apply_transform(rotation_matrix)

        # If position is provided, translate mesh.
        if self.position is not None:
            position_matrix = translation_matrix(self.position)
            self.mesh.apply_transform(position_matrix)

        self.orientation = orientation

    ## API Methods ##
    # Mesh Access #
    @property
    def mesh(self):
        # Prevent access to mesh unless it actually represents region.
        if needsSampling(self) or needsLazyEvaluation(self):
            raise ValueError("Attempting to access the Mesh of an unsampled MeshRegion.")

        return self._mesh

    def findOn(self, point, on_direction):
        """ Find the nearest point in the region following the on_direction.
        Returns None if no such points exist.
        """
        assert not needsSampling(self)

        # Check if this region contains the point, in which case we simply
        # return the point.
        if self.containsPoint(point):
            return point

        # Get first point hit in both directions of ray
        point = point.coordinates
        on_direction = numpy.array(on_direction)

        intersection_data, _, _ = self.mesh.ray.intersects_location(ray_origins=[point, point], \
            ray_directions=[on_direction, -1*on_direction], multiple_hits=False)

        if len(intersection_data) == 0:
            return None

        # Get point with least euclidean distance
        def euclidean_distance(p_1, p_2):
            diff_list = [p_1[i] - p_2[i] for i in range(3)]
            square_list = [math.pow(p, 2) for p in diff_list]
            return math.sqrt(sum(square_list))

        distances = [euclidean_distance(point, p) for p in intersection_data]

        closest_point = intersection_data[distances.index(min(distances))]

        return Vector(*closest_point)

    @cached_property
    def circumcircle(self):
        """ Compute an upper bound on the radius of the region"""
        assert not needsSampling(self)

        center_point = Vector(*self.mesh.bounding_box.center_mass)
        half_extents = [val/2 for val in self.mesh.extents]
        circumradius = hypot(*half_extents)

        return (center_point, circumradius)

class MeshVolumeRegion(MeshRegion):
    """ An instance of MeshRegion that performs operations over the volume of the mesh.

    At intialization time this region assigns the property ``num_samples``,
    which is the number of samples to attempt to get at least a 99% probability
    of one being in the volume. This can be overwritten after initialization time,
    but if it is too low a `RejectionException` is likely and if it is too high sampling
    can be prolonged.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Ensure the mesh is watertight so volume is well defined
        if not self._mesh.is_volume:
            raise ValueError("A MeshVolumeRegion cannot be defined with a mesh that does not have a well defined volume.")

        # Compute how many samples are necessary to achieve 99% probability
        # of success when rejection sampling volume.
        p_volume = self._mesh.volume/self._mesh.bounding_box.volume

        if p_volume > 0.99:
            self.num_samples = 1
        else:
            self.num_samples = min(1e6, max(1, math.ceil(math.log(0.01, 1 - p_volume))))

    # Property testing methods #
    def intersects(self, other, triedReversed=False):
        """Check if this region intersects another.

        This function handles intersect calculations for `MeshVolumeRegion` with:
        * `MeshVolumeRegion`
        * `MeshSurfaceRegion`
        * `PolygonalFootprintRegion`
        """
        # Check if region is fixed, and if not returns a default implementation
        if needsSampling(self):
            return super().intersects(other)

        if isinstance(other, MeshVolumeRegion):
            # PASS 1
            # Check if bounding boxes intersect. If not, volumes cannot intersect.
            # For bounding boxes to intersect there must be overlap of the bounds
            # in all 3 dimensions.
            range_overlaps = [(self.mesh.bounds[0,dim] <= other.mesh.bounds[1,dim]) and \
                              (other.mesh.bounds[0,dim] <= self.mesh.bounds[1,dim]) \
                              for dim in range(3)]
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
            elif len(samples:=trimesh.sample.volume_mesh(self.mesh, self.num_samples)) > 0:
                s_candidate_point = Vector(*samples[0])
            else:
                s_candidate_point = None

            if other.containsPoint(Vector(*other.mesh.bounding_box.center_mass)):
                o_candidate_point = Vector(*other.mesh.bounding_box.center_mass)
            elif len(samples:=trimesh.sample.volume_mesh(other.mesh, other.num_samples)) > 0:
                o_candidate_point = Vector(*samples[0])
            else:
                o_candidate_point = None

            if s_candidate_point is not None and o_candidate_point is not None:
                # Compute the inradius of each object from its candidate point.
                s_inradius = abs(trimesh.proximity.ProximityQuery(self.mesh).signed_distance([s_candidate_point])[0])
                o_inradius = abs(trimesh.proximity.ProximityQuery(other.mesh).signed_distance([o_candidate_point])[0])

                # Compute the circumradius of each object from its candidate point.
                s_circumradius = numpy.max(numpy.linalg.norm(self.mesh.vertices - s_candidate_point, axis=1))
                o_circumradius = numpy.max(numpy.linalg.norm(other.mesh.vertices - o_candidate_point, axis=1))

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
                # Collision manager covers all cases for convex shapes,
                # so we can just return the value.
                return surface_collision

            # PASS 4
            # Compute intersection and check if it's empty. Expensive but guaranteed
            # to give the right answer.
            return not isinstance(self.intersect(other), EmptyRegion)

        if isinstance(other, MeshSurfaceRegion):
            # PASS 1
            # Check if bounding boxes intersect. If not, volumes cannot intersect.
            # For bounding boxes to intersect there must be overlap of the bounds
            # in all 3 dimensions.
            range_overlaps = [(self.mesh.bounds[0,dim] <= other.mesh.bounds[1,dim]) and \
                              (other.mesh.bounds[0,dim] <= self.mesh.bounds[1,dim]) \
                              for dim in range(3)]
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
            collision_manager.add_object("OtherRegion", SurfaceCollisionTrimesh(faces=other.mesh.faces, vertices=other.mesh.vertices))

            surface_collision = collision_manager.in_collision_internal()

            if surface_collision:
                return True

            # PASS 3
            # Compute intersection and check if it's empty. Expensive but guaranteed
            # to give the right answer.
            return not isinstance(self.intersect(other), EmptyRegion)

        if isinstance(other, PolygonalFootprintRegion):
            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            center_z = (vertical_bounds[1] + vertical_bounds[0])/2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(center_z, mesh_height)

            return self.intersects(bounded_footprint)

        if not triedReversed:
            return other.intersects(self)

        raise NotImplementedError(f"Cannot check intersection of MeshRegion with {type(other)}.")

    def containsPoint(self, point):
        """Check if this region's volume contains a point."""
        return self.distanceTo(point) < self.tolerance

    def containsObject(self, obj):
        """Check if this region's volume contains an :obj:`~scenic.core.object_types.Object`.
        The object must support coercion to a mesh.
        """
        # PASS 1
        # Check if bounding boxes intersect. If not, volumes cannot intersect and so
        # the object cannot be contained in this region.
        range_overlaps = [(self.mesh.bounds[0,dim] <= obj.occupiedSpace.mesh.bounds[1,dim]) and \
                          (obj.occupiedSpace.mesh.bounds[0,dim] <= self.mesh.bounds[1,dim]) \
                          for dim in range(3)]
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
        elif len(samples:=trimesh.sample.volume_mesh(obj.occupiedSpace.mesh, obj.occupiedSpace.num_samples)) > 0:
            obj_candidate_point = Vector(*samples[0])
        else:
            obj_candidate_point = None

        if obj_candidate_point is not None:
            # If this region doesn't contain the candidate point, it can't contain the object.
            if not self.containsPoint(obj_candidate_point):
                return False

            # Compute the circumradius of the object from the candidate point.
            obj_circumradius = numpy.max(numpy.linalg.norm(obj.occupiedSpace.mesh.vertices - obj_candidate_point, axis=1))

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
        elif len(samples:=trimesh.sample.volume_mesh(self.mesh, self.num_samples)) > 0:
            reg_candidate_point = Vector(*samples[0])
        else:
            reg_candidate_point = None

        if reg_candidate_point is not None:
            # Calculate circumradius of the region from the candidate_point
            reg_circumradius = numpy.max(numpy.linalg.norm(self.mesh.vertices - reg_candidate_point, axis=1))

            # Calculate maximum distance to the object.
            obj_max_distance = numpy.max(numpy.linalg.norm(obj.occupiedSpace.mesh.vertices - reg_candidate_point, axis=1))

            if obj_max_distance > reg_circumradius:
                return False

        # PASS 5
        # If the difference between the object's region and this region is empty,
        # i.e. obj_region - self_region = EmptyRegion, that means the object is
        # entirely contained in this region.
        diff_region = obj.occupiedSpace.difference(self)

        return isinstance(diff_region, EmptyRegion)


    # Composition methods #
    @lru_cache(maxsize=None)
    def intersect(self, other, triedReversed=False):
        """ Get a `Region` representing the intersection of this region's
        volume with another region.

        This function handles intersection computation for `MeshVolumeRegion` with:
        * `MeshVolumeRegion`
        * `PolygonalFootprintRegion`
        * `PolygonalRegion`
        * `PathRegion`
        * `PolylineRegion`
        """
        # If one of the regions isn't fixed fall back on default behavior
        if needsSampling(self) or needsSampling(other):
            return super().intersect(other, triedReversed)

        if isinstance(other, MeshVolumeRegion):
            # Other region is a mesh volume. We can extract the mesh to perform boolean operations on it
            other_mesh = other.mesh

            # Compute intersection using Trimesh (CalledProcessError usually means empty intersection for OpenSCAD)
            try:
                new_mesh = self.mesh.intersection(other_mesh, engine=self.engine)
            except CalledProcessError:
                if self.engine == "scad":
                    return EmptyRegion("EmptyMesh")
                else:
                    raise RuntimeError("Mesh boolean operation failed.")
            except ValueError as exc:
                raise ValueError("Unable to compute mesh boolean operation. Do you have the Blender and OpenSCAD installed on your system?") from exc

            if new_mesh.is_empty:
                return EmptyRegion("EmptyMesh")
            elif new_mesh.is_volume:
                return MeshVolumeRegion(new_mesh, tolerance=min(self.tolerance, other.tolerance), center_mesh=False, engine=self.engine)
            else:
                # Something went wrong, abort
                return super().intersect(other, triedReversed)

        # WIP Code: Currently correct but VERY slow.
        # if isinstance(other, MeshSurfaceRegion):
        #     # Other region is a mesh surface. We can intersect each triangle and merge them back together.

        #     # Extract triangles from the other mesh
        #     surface_triangles = other.mesh.triangles

        #     # Sort triangles by whether they are fully inside/outside the mesh or intersecting. 
        #     # Triangles fully outside can be discarded, triangles fully inside can be kept unchanged.
        #     # Triangles that intersect the mesh will need to be intersected using an engine.
        #     processed_triangles = []

        #     cm = trimesh.collision.CollisionManager()
        #     cm.add_object("MeshVolume", SurfaceCollisionTrimesh(faces=self.mesh.faces, vertices=self.mesh.vertices))

        #     pq = trimesh.proximity.ProximityQuery(self.mesh)

        #     for i, triangle in enumerate(surface_triangles):
        #         print(f"Triangle {i}/{len(surface_triangles)}")
        #         triangle_mesh = SurfaceCollisionTrimesh(**trimesh.triangles.to_kwargs([triangle]))

        #         # TODO: Compress all these into one call?
        #         collision = cm.in_collision_single(triangle_mesh)

        #         if collision:
        #             # Triangle intersects, compute the intersection and add it.
        #             print("Intersecting...")
        #             # TODO: Speed up this step. Takes a VERY long time
        #             intersected_triangle_mesh = self.mesh.intersection(triangle_mesh)

        #             if not isinstance(intersected_triangle_mesh, trimesh.Trimesh):
        #                 # Triangle doesn't actually intersect
        #                 continue

        #             intersected_triangles = list(intersected_triangle_mesh.triangles)

        #             processed_triangles += intersected_triangles
        #             continue

        #         # Triangle doesn't intersect, check if it's inside or outside the mesh
        #         point_distances = pq.signed_distance(triangle)

        #         if all(point_distances >= 0):
        #             # Triangle is inside, we can keep it unchanged
        #             processed_triangles.append(triangle)
        #             continue
        #         elif all(point_distances <= 0):
        #             # Triangle is outside, we can discard it
        #             continue
        #         else:
        #             # Triangle is intersecting, something went wrong.
        #             assert False, point_distances

        #     # Remake the surface, processing to hopefully clean it up.
        #     new_surface = trimesh.Trimesh(**trimesh.triangles.to_kwargs(processed_triangles), process=True)

        #     return MeshSurfaceRegion(mesh=new_surface, tolerance=min(self.tolerance, other.tolerance), center_mesh=False)

        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a polygonal footprint region. We can bound it in the vertical dimension
            # and then calculate the intersection with the resulting mesh volume.

            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            center_z = (vertical_bounds[1] + vertical_bounds[0])/2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(center_z, mesh_height)

            return self.intersect(bounded_footprint)

        if isinstance(other, PolygonalRegion):
            # Other region can be represented by a polygon. We can slice the volume at the polygon's height,
            # and then take the intersection of the resulting polygons.
            origin_point = (self.mesh.centroid[0], self.mesh.centroid[1], other.z)
            slice_3d = self.mesh.section(plane_origin=origin_point, plane_normal=[0,0,1])

            if slice_3d is None:
                return nowhere

            slice_2d, _ = slice_3d.to_planar(to_2D=numpy.eye(4))
            polygons = MultiPolygon(slice_2d.polygons_full) & other.polygons

            if isinstance(polygons, shapely.geometry.Polygon):
                polygons = MultiPolygon([polygons])

            if polygons.is_empty:
                return nowhere

            return PolygonalRegion(polygon=polygons, z=other.z)

        if isinstance(other, PathRegion):
            # Extract lines from region
            edges = [(other.vert_to_vec[v1], other.vert_to_vec[v2]) for v1,v2 in other.edges]

            # Split lines anytime they cross the mesh boundaries
            refined_polylines = []

            for line_iter, line in enumerate(edges):
                source, dest = line

                ray = dest - source
                ray = ray/numpy.linalg.norm(ray)

                intersections = self.mesh.ray.intersects_location(ray_origins=[source],ray_directions=[ray])[0]

                inner_points = sorted(intersections, key=lambda pos: numpy.linalg.norm(source-pos))
                inner_points = filter(lambda point: numpy.linalg.norm(point-source) < numpy.linalg.norm(dest-source), inner_points)

                refined_points = [source] + list(inner_points) + [dest]

                refined_polylines.append(refined_points)

            # Keep only lines and vertices for line segments in the mesh. 
            internal_lines = []

            for polyline in refined_polylines:
                source = polyline[0]

                for pt_iter in range(1, len(polyline)):
                    dest = polyline[pt_iter]

                    midpoint = (source + dest)/2
                    if self.containsPoint(midpoint):
                        internal_lines.append((source, dest))

                    source = dest

            # Check if merged lines is empty. If so, return the EmptyRegion. Otherwise, 
            # transform merged lines back into a path region.
            if internal_lines:
                return PathRegion(polylines=internal_lines)
            else:
                return EmptyRegion("Empty mesh-polyline merge")

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
                segments = [(v_iter-1, v_iter) for v_iter in range(1,len(vertices))]

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

                    ray = ray/numpy.linalg.norm(ray)

                    intersections = self.mesh.ray.intersects_location(ray_origins=[source],ray_directions=[ray])[0]

                    inner_points = sorted(intersections, key=lambda pos: numpy.linalg.norm(source-pos))

                    for point in inner_points:
                        if numpy.linalg.norm(point-source) < numpy.linalg.norm(dest-source):
                            refined_vertices.append(point)

                    refined_vertices.append(dest)

                refined_segments = [(v_iter-1, v_iter) for v_iter in range(1,len(refined_vertices))]

                refined_lines.append((refined_vertices, refined_segments))

            # Keep only lines and vertices for line segments in the mesh. Also converts them
            # to shapely's point format.
            internal_lines = []

            for vertices, segments in refined_lines:
                for segment in segments:
                    source = vertices[segment[0]]
                    dest = vertices[segment[1]]

                    midpoint = (source + dest)/2

                    if self.containsPoint(midpoint):
                        internal_lines.append((source, dest))

            merged_lines = shapely.ops.linemerge(internal_lines)

            # Check if merged lines is empty. If so, return the EmptyRegion. Otherwise, 
            # transform merged lines back into a polyline region.
            if merged_lines:
                return PolylineRegion(polyline=shapely.ops.linemerge(internal_lines))
            else:
                return EmptyRegion("Empty mesh-polyline merge")

        # Don't know how to compute this intersection, fall back to default behavior.
        return super().intersect(other, triedReversed)

    def union(self, other, triedReversed=False):
        """ Get a `Region` representing the union of this region's
        volume with another region.

        This function handles union computation for `MeshVolumeRegion` with:
            - `MeshVolumeRegion`
        """
        # If one of the regions isn't fixed fall back on default behavior
        if needsSampling(self) or needsSampling(other):
            return super().union(other, triedReversed)

        # If other region is represented by a mesh, we can extract the mesh to
        # perform boolean operations on it
        if isinstance(other, MeshVolumeRegion):
            other_mesh = other.mesh

            # Compute union using Trimesh
            try:
                new_mesh = self.mesh.union(other_mesh, engine=self.engine)
            except ValueError as exc:
                raise ValueError("Unable to compute mesh boolean operation. Do you have the Blender and OpenSCAD installed on your system?") from exc

            if new_mesh.is_empty:
                return EmptyRegion("EmptyMesh")
            elif new_mesh.is_volume:
                return MeshVolumeRegion(new_mesh, tolerance=min(self.tolerance, other.tolerance), center_mesh=False, engine=self.engine)
            else:
                # Something went wrong, abort
                return super().union(other, triedReversed)

        # Don't know how to compute this union, fall back to default behavior.
        return super().union(other, triedReversed)

    def difference(self, other, debug=False):
        """ Get a `Region` representing the difference of this region's
        volume with another region.

        This function handles union computation for `MeshVolumeRegion` with:
        * `MeshVolumeRegion`
        * `PolygonalFootprintRegion`
        """
        # If one of the regions isn't fixed fall back on default behavior
        if needsSampling(self) or needsSampling(other):
            return super().difference(other)

        # If other region is represented by a mesh, we can extract the mesh to
        # perform boolean operations on it
        if isinstance(other, MeshVolumeRegion):
            other_mesh = other.mesh

            # Compute difference using Trimesh (CalledProcessError usually means empty intersection for OpenSCAD)
            try:
                new_mesh = self.mesh.difference(other_mesh, engine=self.engine, debug=debug)
            except CalledProcessError:
                if self.engine == "scad":
                    return EmptyRegion("EmptyMesh")
                else:
                    raise RuntimeError("Mesh boolean operation failed.")
            except ValueError as exc:
                raise ValueError("Unable to compute mesh boolean operation. Do you have the Blender and OpenSCAD installed on your system?") from exc

            if new_mesh.is_empty:
                return EmptyRegion("EmptyMesh")
            elif new_mesh.is_volume:
                return MeshVolumeRegion(new_mesh, tolerance=min(self.tolerance, other.tolerance), center_mesh=False, engine=self.engine)
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
            center_z = (vertical_bounds[1] + vertical_bounds[0])/2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(center_z, mesh_height)

            return self.difference(bounded_footprint)

        # Don't know how to compute this difference, fall back to default behavior.
        return super().difference(other)


    def uniformPointInner(self):
        """ Samples a point uniformly from the volume of the region"""
        # TODO: Look into tetrahedralization, perhaps to be turned on when a heuristic
        # is met. Currently using Trimesh's rejection sampling.
        sample = trimesh.sample.volume_mesh(self.mesh, self.num_samples)

        if len(sample) == 0:
            raise RejectionException("Rejection sampling MeshVolumeRegion failed.")
        else:
            return Vector(*sample[0])

    def distanceTo(self, point):
        """ Get the minimum distance from this region (including volume) to the specified point."""
        point = toVector(point, "Could not convert 'point' to vector.")

        pq = trimesh.proximity.ProximityQuery(self.mesh)

        dist = pq.signed_distance([point.coordinates])[0]

        # Positive distance indicates being contained in the mesh.
        if dist > 0:
            dist = 0

        return abs(dist)

    @cached_property
    def inradius(self):
        assert not needsSampling(self)
        center_point = self.mesh.bounding_box.center_mass

        pq = trimesh.proximity.ProximityQuery(self.mesh)
        region_distance = abs(pq.signed_distance([center_point])[0])

        if region_distance < 0:
            return 0
        else:
            return region_distance

    @cached_property
    def isConvex(self):
        return self.mesh.is_convex

    @property
    def dimensionality(self):
        return 3

    @cached_property
    def size(self):
        return self.mesh.mass/self.mesh.density

    ## Sampling Methods ##
    def sampleGiven(self, value):
        return MeshVolumeRegion(mesh=value[self._mesh], name=self.name, \
            dimensions=value[self.dimensions], position=value[self.position], rotation=value[self.rotation], \
            orientation=self.orientation, tolerance=self.tolerance, \
            center_mesh=self.center_mesh, engine=self.engine)

    ## Utility Methods ##
    @lru_cache(maxsize=None)
    def getSurfaceRegion(self):
        """ Return a region equivalent to this one, except as a MeshSurfaceRegion"""
        return MeshSurfaceRegion(self.mesh, self.name, orientation=self.orientation, \
            tolerance=self.tolerance, center_mesh=False, engine=self.engine)

    def getVolumeRegion(self):
        """ Returns this object, as it is already a MeshVolumeRegion"""
        return self


class MeshSurfaceRegion(MeshRegion):
    """ An instance of MeshRegion that performs operations over the surface
    of the mesh.

    If an orientation is not passed to this mesh, a default orientation is
    provided which provides an orientation that aligns an instance's z axis 
    with the normal vector of the face containing that point.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Set default orientation to one inferred from face norms if none is provided.
        if self.orientation is None:
            self.orientation = VectorField("DefaultSurfaceVectorField", lambda pos: self.getFlatOrientation(pos))

    # Property testing methods #
    def intersects(self, other, triedReversed=False):
        """ Check if this region's surface intersects another region.
        This is equivalent to checking if the two meshes collide

        This function handles intersection computation for `MeshSurfaceRegion` with:
        * `MeshSurfaceRegion`
        * `PolygonalFootprintRegion`
        """
        # Check if region is fixed, and if not returns a default implementation
        if needsSampling(self):
            return super().intersects(other)

        elif isinstance(other, MeshSurfaceRegion):
            # Uses Trimesh's collision manager to check for intersection of the
            # surfaces. Use SurfaceCollisionTrimesh objects to ensure collisions
            # actually imply a surface collision.
            collision_manager = trimesh.collision.CollisionManager()

            collision_manager.add_object("SelfRegion", SurfaceCollisionTrimesh(faces=self.mesh.faces, vertices=self.mesh.vertices))
            collision_manager.add_object("OtherRegion", SurfaceCollisionTrimesh(faces=other.mesh.faces, vertices=other.mesh.vertices))

            surface_collision = collision_manager.in_collision_internal()

            return surface_collision

        if isinstance(other, PolygonalFootprintRegion):
            # Determine the mesh's vertical bounds (adding a little extra to avoid mesh errors) and
            # the mesh's vertical center.
            vertical_bounds = (self.mesh.bounds[0][2], self.mesh.bounds[1][2])
            mesh_height = vertical_bounds[1] - vertical_bounds[0] + 1
            center_z = (vertical_bounds[1] + vertical_bounds[0])/2

            # Compute the bounded footprint and recursively compute the intersection
            bounded_footprint = other.approxBoundFootprint(center_z, mesh_height)

            return self.intersects(bounded_footprint)

        elif not triedReversed:
            return other.intersects(self)

        raise NotImplementedError("Cannot check intersection of MeshRegion with " +
            type(other) + ".")

    def containsPoint(self, point):
        """Check if this region's surface contains a point."""
        # First compute the minimum distance to the point.
        min_distance = self.distanceTo(point)

        # If the minimum distance is within tolerance of 0, the mesh contains the point.
        return min_distance < self.tolerance

    def containsObject(self, obj):
        # A surface cannot contain an object, which must have a volume.
        return False

    def uniformPointInner(self):
        """ Sample a point uniformly at random from the surface of them mesh"""
        return Vector(*trimesh.sample.sample_surface(self.mesh, 1)[0][0])

    def distanceTo(self, point):
        """ Get the minimum distance from this object to the specified point."""
        point = toVector(point, "Could not convert 'point' to vector.")

        pq = trimesh.proximity.ProximityQuery(self.mesh)

        dist = abs(pq.signed_distance([point.coordinates])[0])

        return dist

    @property
    def dimensionality(self):
        return 2

    @cached_property
    def size(self):
        return self.mesh.area

    def getFlatOrientation(self, pos):
        """Given a point on the surface of the mesh, return an orientation that aligns
        an instance's z axis with the normal vector of the face containing that point.

        If ``pos`` is not within ``self.tolerance`` of the surface of the mesh, a
        ``RejectionException`` is raised.
        """
        prox_query = trimesh.proximity.ProximityQuery(self.mesh)

        _, distance, triangle_id = prox_query.on_surface([pos.coordinates])

        if distance > self.tolerance:
            raise RejectionException("Attempted to get flat orientation of a mesh away from a surface.")

        face_normal_vector = self.mesh.face_normals[triangle_id][0]

        transform = trimesh.geometry.align_vectors([0,0,1], face_normal_vector)
        orientation = tuple(scipy.spatial.transform.Rotation.from_matrix(transform[:3,:3]).as_euler('ZXY'))

        return orientation

    ## Sampling Methods ##
    def sampleGiven(self, value):
        return MeshSurfaceRegion(mesh=value[self._mesh], name=self.name, \
            dimensions=value[self.dimensions], position=value[self.position], rotation=value[self.rotation], \
            orientation=self.orientation, tolerance=self.tolerance, \
            center_mesh=self.center_mesh, engine=self.engine)

    ## Utility Methods ##
    @lru_cache(maxsize=None)
    def getVolumeRegion(self):
        """ Return a region equivalent to this one, except as a MeshVolumeRegion"""
        return MeshVolumeRegion(self.mesh, self.name, orientation=self.orientation, \
            tolerance=self.tolerance, center_mesh=False, engine=self.engine)

    def getSurfaceRegion(self):
        """ Returns this object, as it is already a MeshSurfaceRegion"""
        return self

class BoxRegion(MeshVolumeRegion):
    """ Region in the shape of a rectangular cuboid, i.e. a box. By default the unit box centered at the origin
    and aligned with the axes is used.

    Args:
        name: An optional name to help with debugging.
        dimensions: An optional 3-tuple, with the values representing width, length, height respectively.
          The mesh will be scaled such that the bounding box for the mesh has these dimensions.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for collision computations.
        engine: Which engine to use for mesh operations. Either "blender" or "scad".
    """
    def __init__(self, name=None, dimensions=None, position=None, rotation=None, orientation=None, tolerance=1e-8, engine="blender"):
        box_mesh = trimesh.creation.box((1, 1, 1))
        super().__init__(mesh=box_mesh, name=name, position=position, rotation=rotation, dimensions=dimensions,\
        orientation=orientation, tolerance=tolerance, engine=engine)

    @cached_property
    def isConvex(self):
        return True

class SpheroidRegion(MeshVolumeRegion):
    """ Region in the shape of a spheroid. By default the unit sphere centered at the origin
    and aligned with the axes is used.

    Args:
        name: An optional name to help with debugging.
        dimensions: An optional 3-tuple, with the values representing width, length, height respectively.
          The mesh will be scaled such that the bounding box for the mesh has these dimensions.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for collision computations.
        engine: Which engine to use for mesh operations. Either "blender" or "scad".
    """
    def __init__(self, name=None, dimensions=None, position=None, rotation=None, orientation=None, tolerance=1e-8, engine="blender"):
        sphere_mesh = trimesh.creation.icosphere(radius=1)
        super().__init__(mesh=sphere_mesh, name=name, position=position, rotation=rotation, dimensions=dimensions, \
            orientation=orientation, tolerance=tolerance, engine=engine)

    @cached_property
    def isConvex(self):
        return True

class PolygonalFootprintRegion(Region):
    """ Region that contains all points in a polygonal footprint, regardless of their z value. This region
    cannot be sampled from, as it has infinite height and therefore infinite volume.
    
    Args:
        polygon: A ``MultiPolygon`` that defines the footprint of this region.
        name: An optional name to help with debugging.
    """
    def __init__(self, polygon, name=None):
        if not isinstance(polygon, MultiPolygon):
            raise RuntimeError("'polygon' must be a shapely MultiPolygon")

        self.polygon = polygon
        super().__init__(name)
        self._bounded_cache = None

    def intersect(self, other, triedReversed=False):
        """ Get a `Region` representing the intersection of this region's
        volume with another region.

        This function handles intersection computation for `PolygonalFootprintRegion` with:
        * `PolygonalFootprintRegion`
        * `PolygonalRegion`
        """
        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a PolygonalFootprintRegion, so we can just intersect the base polygons
            # and take the footprint of the result, if it isn't empty.
            new_poly = PolygonalRegion(polygon=self.polygon).intersect(PolygonalRegion(polygon=other.polygon))

            if isinstance(new_poly, EmptyRegion):
                return new_poly
            else:
                return new_poly.footprint

        if isinstance(other, PolygonalRegion):
            # Other region can be represented by a polygon. We can take the intersection of that
            # polygon with our base polygon at the correct z position, and then output the resulting polygon.
            return PolygonalRegion(polygon=self.polygon, z=other.z).intersect(other)

        return super().intersect(other, triedReversed)

    def union(self, other, triedReversed=False):
        """ Get a `Region` representing the intersection of this region's
        volume with another region.

        This function handles union computation for `PolygonalFootprintRegion` with:
        * `PolygonalFootprintRegion`
        """
        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a PolygonalFootprintRegion, so we can just union the base polygons
            # and take the footprint of the result.
            return PolygonalRegion(polygon=self.polygon).union(PolygonalRegion(polygon=other.polygon)).footprint

        return super().union(other, triedReversed)

    def difference(self, other):
        """ Get a `Region` representing the intersection of this region's
        volume with another region.

        This function handles difference computation for `PolygonalFootprintRegion` with:
        * `PolygonalFootprintRegion`
        """
        if isinstance(other, PolygonalFootprintRegion):
            # Other region is a PolygonalFootprintRegion, so we can just difference the base polygons
            # and take the footprint of the result, if it isn't empty.
            new_poly = PolygonalRegion(polygon=self.polygon).difference(PolygonalRegion(polygon=other.polygon))

            if isinstance(new_poly, EmptyRegion):
                return new_poly
            else:
                return new_poly.footprint

        return super().difference(other)

    def uniformPointInner(self):
        raise UndefinedSamplingException(f'Attempted to sample from a PolygonalFootprintRegion, for which uniform sampling is undefined')

    def containsPoint(self, point):
        """ Checks if a point is contained in the polygonal footprint by checking
        if the (x, y) values are contained in the polygon.

        Args:
            point: A point to be checked for containment.
        """
        x_y_point = toVector(point)[:2]
        return self.polygon.intersects(shapely.geometry.Point(x_y_point))

    def containsObject(self, obj):
        """ Checks if an object is contained in the polygonal footprint.
         The polygonal footprint is bounded so that it contains all vertical values of the object. 
        Then we check if the object is contained in this volume.

        Args:
            obj: An object to be checked for containment.
        """
        # PASS 1
        # If the polygon is convex then so is this region, so we can first check if all
        # all points of the object's bounding box are contained in the region. If so
        # the object must be contained
        if all(self.containsPoint(corner) for corner in obj.corners):
            return True

        # PASS 2
        # Compute a bounded footprint that covers the object and then check containment
        # directly.

        # Determine the mesh's vertical bounds, and calculate height equal to the mesh's
        # (plus a little extra) along with the central z point of the mesh.
        vertical_bounds = (obj.occupiedSpace.mesh.bounds[0][2], obj.occupiedSpace.mesh.bounds[1][2])
        height = vertical_bounds[1] - vertical_bounds[0] + 1
        center_z = (vertical_bounds[1] + vertical_bounds[0])/2

        # Create a bounded footprint of the mesh.
        bounded_footprint = self.approxBoundFootprint(center_z, height)

        # Check for containment of the object in the bounded footprint.
        return bounded_footprint.containsObject(obj)

    def approxBoundFootprint(self, center_z, height):
        """ Returns a boundFootprint volume that is guaranteed to contain the
        result of boundFootprint(center_z, height), but may be taller. Used 
        to save time on recomputing boundFootprint.
        """
        if self._bounded_cache is not None:
            # See if we can reuse a previous region
            prev_center_z, prev_height, prev_bounded_footprint = self._bounded_cache

            if prev_center_z + prev_height/2 > center_z + height/2 and \
               prev_center_z - prev_height/2 < center_z - height/2:
                # Cached region covers requested region, so return that.
                return prev_bounded_footprint

        # Populate cache, and make the height bigger than requested to try to
        # save on future calls.
        padded_height = 100*max(1,center_z)*height
        bounded_footprint = self.boundFootprint(center_z, padded_height)

        self._bounded_cache = center_z, padded_height, bounded_footprint

        return bounded_footprint

    def boundFootprint(self, center_z, height):
        """ Cap the footprint of the object to a given height, centered at a
        given z.

        Args:
            center_z: The resulting mesh will be vertically centered at this height.
            height: The resulting mesh will have this height.
        """
        # Fall back on progressively higher buffering and simplification to 
        # get the mesh to be a valid volume
        tol_sizes = [None, 0.00001, 0.0001, 0.001, 0.01, 0.1, 1]

        for tol_size in tol_sizes:
            # If needed, buffer the multipolygon
            if tol_size is None:
                poly_obj = self.polygon
            else:
                poly_obj = self.polygon.buffer(tol_size).simplify(tol_size)

            # Extract a list of the polygon
            if isinstance(poly_obj, shapely.geometry.Polygon):
                polys = [poly_obj]
            else:
                polys = list(poly_obj.geoms)

            # Extrude the polygon to the desired height
            vf = [trimesh.creation.triangulate_polygon(p, engine='earcut') for p in polys]
            v, f = trimesh.util.append_faces([i[0] for i in vf], [i[1] for i in vf])
            polygon_mesh = trimesh.creation.extrude_triangulation(vertices=v, faces=f, height=height)

            # Translate the polygon mesh to the desired height.
            polygon_mesh.vertices[:,2] += center_z - polygon_mesh.bounding_box.center_mass[2]

            # Check if we have a valid volume
            if polygon_mesh.is_volume:
                if tol_size is not None:
                    warnings.warn(f"Computing bounded footprint of polygon resulted in invalid volume, "
                                  f"but was able to remedy this by buffering/simplifying polygon by {tol_size}")
                break

        if not polygon_mesh.is_volume:
            raise RuntimeError("Computing bounded footprint of polygon resulted in invalid volume")

        return MeshVolumeRegion(polygon_mesh, center_mesh=False)

    def buffer(self, amount):
        buffered_polygon = self.polygon.buffer(amount)

        if isinstance(buffered_polygon, shapely.geometry.Polygon):
            buffered_polygon = MultiPolygon([buffered_polygon])

        return PolygonalFootprintRegion(polygon=buffered_polygon, name=self.name)

    @cached_property
    def isConvex(self):
        return self.polygons.equals(self.polygons.convex_hull)
    
class PathRegion(Region):
    def __init__(self, points=None, polylines=None):
        """ A region composed of multiple polylines in 3D space.

        One of points or polylines should be provided. If both are provided,
        points will be used.

        Args:
            points: A list of points defining a single polyline.
            polylines: A list of list of points, defining multiple polylines.
        """
        # Standardize inputs
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

        self.vert_to_vec = {val:key for key,val in self.vec_to_vert.items()}
        self.vertices = list(sorted(self.vec_to_vert.keys(), key=lambda vec: self.vec_to_vert[vec]))

        # TODO: Detect where lines intersect to remove duplication
        warnings.warn("Currently PathRegion does not check for overlapping line segments."
                      "If points or polylines has overlap, sampling a point will not be exactly uniform.")

        # Extract length of each edge
        self.edge_lengths = []

        for edge in self.edges:
            v1, v2 = edge
            c1, c2 = self.vert_to_vec[v1], self.vert_to_vec[v2]

            self.edge_lengths.append(c1.distanceTo(c2))

    @lru_cache(maxsize=None)
    def containsPoint(self, point, epsilon=1e-8):
        pt = toVector(point)

        # Check if each edge contains the point
        for edge_iter, edge in enumerate(self.edges):
            v1, v2 = edge
            c1, c2 = self.vert_to_vec[v1], self.vert_to_vec[v2]

            if (c1.distanceTo(pt) + pt.distanceTo(c2)) <= c1.distanceTo(c2) + epsilon:
                return True

        return False

    def containsObject(self, obj):
        return False

    def uniformPointInner(self):
        # Pick an edge, weighted by length, and extract its two points
        edge = random.choices(population=self.edges, weights=self.edge_lengths, k=1)[0]
        v1, v2 = edge
        c1, c2 = self.vert_to_vec[v1], self.vert_to_vec[v2]

        # Sample uniformly from the line segment
        sampled_pt = c1 + random.uniform(0,1)*(c2 - c1)

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
    """Region given by one or more polygons (possibly with holes).

    The region may be specified by giving either a sequence of points defining the
    boundary of the polygon, or a collection of ``shapely`` polygons (a ``Polygon``
    or ``MultiPolygon``).

    Args:
        points: sequence of points making up the boundary of the polygon (or `None` if
            using the **polygon** argument instead).
        polygon: ``shapely`` polygon or collection of polygons (or `None` if using
            the **points** argument instead).
        orientation (`VectorField`; optional): :term:`preferred orientation` to use.
        name (str; optional): name for debugging.
    """
    def __init__(self, points=None, polygon=None, orientation=None, name=None, z=0, additional_deps=[]):
        super().__init__(name, *additional_deps, orientation=orientation)

        self.z = z

        # If our polygons aren't defined yet, then wait till later
        if needsSampling(self) or needsLazyEvaluation(self):
            return

        if polygon is None and points is None:
            raise ValueError('must specify points or polygon for PolygonalRegion')
        if polygon is None:
            points = tuple(pt[:2] for pt in points)
            if len(points) == 0:
                raise ValueError('tried to create PolygonalRegion from empty point list!')
            for point in points:
                if needsSampling(point):
                    raise ValueError('only fixed PolygonalRegions are supported')
            self.points = points
            polygon = shapely.geometry.Polygon(points)

        if isinstance(polygon, shapely.geometry.Polygon):
            self.polygons = shapely.geometry.MultiPolygon([polygon])
        elif isinstance(polygon, shapely.geometry.MultiPolygon):
            self.polygons = polygon
        else:
            raise ValueError(f'tried to create PolygonalRegion from non-polygon {polygon}')
        if not self.polygons.is_valid:
            raise ValueError('tried to create PolygonalRegion with '
                             f'invalid polygon {self.polygons}')

        if (points is None and len(self.polygons.geoms) == 1
            and len(self.polygons.geoms[0].interiors) == 0):
            self.points = tuple(self.polygons.geoms[0].exterior.coords[:-1])

        if self.polygons.is_empty:
            raise ValueError('tried to create empty PolygonalRegion')

        triangles = []
        for polygon in self.polygons.geoms:
            triangles.extend(triangulatePolygon(polygon))
        assert len(triangles) > 0, self.polygons
        self.trianglesAndBounds = tuple((tri, tri.bounds) for tri in triangles)
        areas = (triangle.area for triangle in triangles)
        self.cumulativeTriangleAreas = tuple(itertools.accumulate(areas))

    @cached_property
    def footprint(self):
        return PolygonalFootprintRegion(self.polygons)

    def containsPoint(self, point):
        return self.footprint.containsPoint(point)

    def containsObject(self, obj):
        return self.footprint.containsObject(obj)

    def uniformPointInner(self):
        triangle, bounds = random.choices(
            self.trianglesAndBounds,
            cum_weights=self.cumulativeTriangleAreas)[0]
        minx, miny, maxx, maxy = bounds
        # TODO improve?
        while True:
            x, y = random.uniform(minx, maxx), random.uniform(miny, maxy)
            if triangle.intersects(shapely.geometry.Point(x, y)):
                return self.orient(Vector(x, y, self.z))

    def intersects(self, other, triedReversed=False):
        if isinstance(other, PolygonalRegion):
            if self.z != other.z:
                return False

        poly = toPolygon(other)
        if poly is not None:
            intersection = self.polygons & poly
            return not intersection.is_empty
        return super().intersects(other, triedReversed)

    def intersect(self, other, triedReversed=False):
        # If one of the regions isn't fixed fall back on default behavior
        if needsSampling(self) or needsSampling(other):
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
                    geoms = [geom for geom in intersection.geoms if isinstance(geom, poly_geoms)]
                elif intersection.length > 0:
                    line_geoms = (shapely.geometry.LineString, shapely.geometry.MultiLineString)
                    geoms = [geom for geom in intersection.geoms if isinstance(geom, line_geoms)]
                intersection = shapely.ops.unary_union(geoms)
            orientation = orientationFor(self, other, triedReversed)
            return regionFromShapelyObject(intersection, orientation=orientation)
        return super().intersect(other, triedReversed)

    def union(self, other, triedReversed=False, buf=0):
        if isinstance(other, PolygonalRegion):
            if self.z != other.z:
                return super().union(other, triedReversed)

        poly = toPolygon(other)

        if not poly:
            return super().union(other, triedReversed)
        union = polygonUnion((self.polygons, poly), buf=buf)
        orientation = VectorField.forUnionOf((self, other))
        return PolygonalRegion(polygon=union, orientation=orientation)

    def difference(self, other):
        # If one of the regions isn't fixed fall back on default behavior
        if needsSampling(self) or needsSampling(other):
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
    def unionAll(regions, buf=0):
        if not all([r.z == regions[0].z for r in regions]):
            raise ValueError("union of PolygonalRegions with different z values is undefined.")

        regs, polys = [], []
        for reg in regions:
            if reg != nowhere:
                regs.append(reg)
                polys.append(toPolygon(reg))
        if not polys:
            return nowhere
        if any(not poly for poly in polys):
            raise TypeError(f'cannot take union of regions {regions}')
        union = polygonUnion(polys, buf=buf)
        orientation = VectorField.forUnionOf(regs)
        return PolygonalRegion(polygon=union, orientation=orientation)

    @property
    def boundary(self) -> "PolylineRegion":
        """Get the boundary of this region as a `PolylineRegion`."""
        return PolylineRegion(polyline=self.polygons.boundary)

    @cached_property
    def prepared(self):
        return shapely.prepared.prep(self.polygons)

    def containsRegion(self, other, tolerance=0):
        if isinstance(other, EmptyRegion):
            return True
        poly = toPolygon(other)
        if poly is None:
            raise TypeError(f'cannot test inclusion of {other} in PolygonalRegion')
        return self.polygons.buffer(tolerance).contains(poly)

    @distributionMethod
    def distanceTo(self, point):
        # TODO: Fix for 3D
        return self.polygons.distance(shapely.geometry.Point(point))

    def getAABB(self):
        xmin, ymin, xmax, ymax = self.polygons.bounds
        return ((xmin, ymin), (xmax, ymax), (self.z, self.z))

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

    def show(self, plt, style='r-', **kwargs):
        plotPolygon(self.polygons, plt, style=style, **kwargs)

    def __repr__(self):
        return f'PolygonalRegion({self.polygons!r})'

    def __eq__(self, other):
        if type(other) is not PolygonalRegion:
            return NotImplemented
        return (other.polygons == self.polygons
                and other.orientation == self.orientation)

    @cached
    def __hash__(self):
        # TODO better way to hash mutable Shapely geometries? (also for PolylineRegion)
        return hash((str(self.polygons), self.orientation))

    def __getstate__(self):
        state = self.__dict__.copy()
        state.pop('_cached_prepared', None)     # prepared geometries are not picklable
        return state

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
        self.circumcircle = (self.center, self.radius)
        self.resolution = resolution

        deps = [self.center, self.radius]

        if needsSampling(deps):
            # Center and radius aren't fixed, so we'll wait to pass a polygon
            super().__init__(polygon=None, name=name, additional_deps=deps)
            return

        ctr = shapely.geometry.Point(self.center)
        polygons = MultiPolygon([ctr.buffer(self.radius, resolution=self.resolution)])
        super().__init__(polygon=polygons, z=self.center.z, name=name, additional_deps=deps)

    def sampleGiven(self, value):
        return CircularRegion(value[self.center], value[self.radius],
                              name=self.name, resolution=self.resolution)

    def evaluateInner(self, context):
        center = valueInContext(self.center, context)
        radius = valueInContext(self.radius, context)
        return CircularRegion(center, radius, name=self.name, resolution=self.resolution)

    def intersects(self, other, triedReversed=False):
        if isinstance(other, CircularRegion):
            return self.center.distanceTo(other.center) <= self.radius + other.radius
        
        return super().intersects(other)

    def containsPoint(self, point):
        point = toVector(point)

        if point.z != self.z:
            return False

        return point.distanceTo(self.center) <= self.radius

    def distanceTo(self, point):
        # TODO: Fix for 3D
        point = toVector(point)
        return max(0, point.distanceTo(self.center) - self.radius)

    def uniformPointInner(self):
        x, y, z = self.center
        r = random.triangular(0, self.radius, self.radius)
        t = random.uniform(-math.pi, math.pi)
        pt = Vector(x + (r * cos(t)), y + (r * sin(t)), z)
        return self.orient(pt)

    def getAABB(self):
        x, y, _ = self.center
        r = self.radius
        return ((x - r, y - r), (x + r, y + r), (self.z,self.z))

    def __repr__(self):
        return f'CircularRegion({self.center!r}, {self.radius!r})'

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

        if needsSampling(deps) or needsLazyEvaluation(deps):
            # Center and radius aren't fixed, so we'll wait to pass a polygon
            super().__init__(polygon=None, name=name, additional_deps=deps)
            return

        polygons = self._make_sector_polygons()
        super().__init__(polygon=polygons, z=self.center.z, name=name, additional_deps=deps)

    def _make_sector_polygons(self):
        center, radius = self.center, self.radius
        assert not needsSampling((center,radius))
        ctr = shapely.geometry.Point(center)
        circle = ctr.buffer(radius, resolution=self.resolution)
        if self.angle >= math.tau - 0.001:
            polygon = circle
        else:
            heading = self.heading
            half_angle = self.angle / 2
            mask = shapely.geometry.Polygon([
                center,
                center.offsetRadially(radius, heading + half_angle),
                center.offsetRadially(2*radius, heading),
                center.offsetRadially(radius, heading - half_angle)
            ])
            polygon = circle & mask

        return MultiPolygon([polygon])

    def sampleGiven(self, value):
        return SectorRegion(value[self.center], value[self.radius],
            value[self.heading], value[self.angle],
            name=self.name, resolution=self.resolution)

    def evaluateInner(self, context):
        center = valueInContext(self.center, context)
        radius = valueInContext(self.radius, context)
        heading = valueInContext(self.heading, context)
        angle = valueInContext(self.angle, context)
        return SectorRegion(center, radius, heading, angle,
                            name=self.name, resolution=self.resolution)

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
        return f'SectorRegion({self.center!r},{self.radius!r},{self.heading!r},{self.angle!r})'

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
        self.radius = hypot(hw, hl)     # circumcircle; for collision detection
        self.corners = tuple(self.position.offsetRotated(heading, Vector(*offset))
            for offset in ((hw, hl), (-hw, hl), (-hw, -hl), (hw, -hl)))
        self.circumcircle = (self.position, self.radius)

        if needsSampling(deps) or needsLazyEvaluation(deps):
            # Center and radius aren't fixed, so we'll wait to pass a polygon
            super().__init__(polygon=None, name=name, additional_deps=deps)
            return

        polygons = self._make_rectangle_polygons()
        super().__init__(polygon=polygons, z=self.position.z, name=name, additional_deps=deps)

    def _make_rectangle_polygons(self):
        position, heading, hw, hl = self.position, self.heading, self.hw, self.hl
        assert not any(needsSampling(c) or needsLazyEvaluation(c) for c in (position, heading, hw, hl))
        corners = _RotatedRectangle.makeCorners(position.x, position.y, heading, hw, hl)
        polygon = shapely.geometry.Polygon(corners)
        return MultiPolygon([polygon])


    def sampleGiven(self, value):
        return RectangularRegion(value[self.position], value[self.heading],
            value[self.width], value[self.length],
            name=self.name)

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
        pt = self.position.offsetRotated(self.heading, Vector(rx, ry, self.position.z))
        return self.orient(pt)

    def getAABB(self):
        x, y, z = zip(*self.corners)
        minx, maxx = findMinMax(x)
        miny, maxy = findMinMax(y)
        return ((minx, miny), (maxx, maxy), (self.z, self.z))

    def __repr__(self):
        return f'RectangularRegion({self.position!r},{self.heading!r},{self.width!r},{self.length!r})'

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
            orientation = VectorField('Polyline', self.defaultOrientation)
            self.usingDefaultOrientation = True
        else:
            self.usingDefaultOrientation = False
        super().__init__(name, orientation=orientation)

        if points is not None:
            points = tuple(toVector(pt).coordinates for pt in points)

            if any(point[2] != 0 for point in points):
                warnings.warn("'points' passed to PolylineRegion contain non 0 z component values. These will be replaced with 0.")

            points = tuple((x,y,0) for x,y,_ in points)

            if len(points) < 2:
                raise ValueError('tried to create PolylineRegion with < 2 points')
            self.points = points
            self.lineString = shapely.geometry.LineString(points)
        elif polyline is not None:
            if isinstance(polyline, shapely.geometry.LineString):
                if len(polyline.coords) < 2:
                    raise ValueError('tried to create PolylineRegion with <2-point LineString')
            elif isinstance(polyline, shapely.geometry.MultiLineString):
                if len(polyline.geoms) == 0:
                    raise ValueError('tried to create PolylineRegion from empty MultiLineString')
                for line in polyline.geoms:
                    assert len(line.coords) >= 2
            else:
                raise ValueError('tried to create PolylineRegion from non-LineString')
            self.lineString = polyline
            self.points = None
        else:
            raise ValueError('must specify points or polyline for PolylineRegion')

        if not self.lineString.is_valid:
            raise ValueError('tried to create PolylineRegion with '
                             f'invalid LineString {self.lineString}')
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
                raise ValueError('LineString has fewer than 2 points')
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
            raise ValueError('called segmentsOf on non-linestring')

    @cached_property
    def start(self):
        """Get an `OrientedPoint` at the start of the polyline.

        The OP's heading will be aligned with the orientation of the region, if
        there is one (the default orientation pointing along the polyline).
        """
        pointA, pointB = self.segments[0]
        if self.usingDefaultOrientation:
            heading = headingOfSegment(pointA, pointB)
        elif self.orientation is not None:
            heading = self.orientation[Vector(*pointA)].yaw
        else:
            heading = 0
        from scenic.core.object_types import OrientedPoint
        return OrientedPoint(position=pointA, yaw=heading)

    @cached_property
    def end(self):
        """Get an `OrientedPoint` at the end of the polyline.

        The OP's heading will be aligned with the orientation of the region, if
        there is one (the default orientation pointing along the polyline).
        """
        pointA, pointB = self.segments[-1]
        if self.usingDefaultOrientation:
            heading = headingOfSegment(pointA, pointB)
        elif self.orientation is not None:
            heading = self.orientation[Vector(*pointB)].yaw
        else:
            heading = 0
        from scenic.core.object_types import OrientedPoint
        return OrientedPoint(position=pointB, yaw=heading)

    def defaultOrientation(self, point):
        start, end = self.nearestSegmentTo(point)
        return start.angleTo(end)

    def uniformPointInner(self):
        pointA, pointB = random.choices(self.segments,
                                        cum_weights=self.cumulativeLengths)[0]
        interpolation = random.random()
        x, y = averageVectors(pointA, pointB, weight=interpolation)
        if self.usingDefaultOrientation:
            return OrientedVector(x, y, 0, headingOfSegment(pointA, pointB))
        else:
            return self.orient(Vector(x, y, 0))

    def intersect(self, other, triedReversed=False):
        poly = toPolygon(other)
        if poly is not None:
            intersection = self.lineString & poly
            line_geoms = (shapely.geometry.LineString, shapely.geometry.MultiLineString)
            if (isinstance(intersection, shapely.geometry.GeometryCollection)
                and intersection.length > 0):
                geoms = [geom for geom in intersection.geoms if isinstance(geom, line_geoms)]
                intersection = shapely.ops.unary_union(geoms)
            orientation = orientationFor(self, other, triedReversed)
            return regionFromShapelyObject(intersection, orientation=orientation)
        return super().intersect(other, triedReversed)

    def intersects(self, other, triedReversed=False):
        poly = toPolygon(other)
        if poly is not None:
            intersection = self.lineString & poly
            return not intersection.is_empty
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
            raise TypeError(f'cannot take Polyline union of regions {regions}')
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

    def containsPoint(self, point):
        return self.lineString.intersects(shapely.geometry.Point(point))

    def containsObject(self, obj):
        return False

    @distributionMethod
    def distanceTo(self, point) -> float:
        return self.lineString.distance(shapely.geometry.Point(point))

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
        pt = shapely.ops.nearest_points(self.lineString, shapely.geometry.Point(point))[0]
        return Vector(*pt.coords[0])

    @distributionMethod
    def nearestSegmentTo(self, point):
        dist = self.lineString.project(shapely.geometry.Point(point))
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

    @property
    def dimensionality(self):
        return 1

    @cached_property
    def size(self):
        return self.lineString.length

    @property
    def length(self):
        return self.size

    def pointsSeparatedBy(self, distance):
        return [self.pointAlongBy(d) for d in numpy.arange(0, self.length, distance)]

    def getAABB(self):
        xmin, ymin, xmax, ymax = self.lineString.bounds
        return ((xmin, ymin), (xmax, ymax), (0, 0))

    def show(self, plt, style='r-', **kwargs):
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
        return f'PolylineRegion({self.lineString!r})'

    def __eq__(self, other):
        if type(other) is not PolylineRegion:
            return NotImplemented
        return (other.lineString == self.lineString)

    @cached
    def __hash__(self):
        return hash(str(self.lineString))

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
            warnings.warn("Passing a kdTree to the PointSetRegion is deprecated."
                "The value will be ignored and the parameter removed in future versions.",
                DeprecationWarning)

        for point in points:
            if needsSampling(point):
                raise ValueError('only fixed PointSetRegions are supported')
        
        self.points = numpy.array(points)

        if self.points.shape[1] == 2:
            self.points = numpy.hstack((self.points, numpy.zeros((len(self.points),1))))
        elif self.points.shape[1] == 3:
            pass
        else:
            raise ValueError(f"The points parameter had incorrect shape {points.shape}")

        assert self.points.shape[1] == 3

        import scipy.spatial    # slow import not often needed
        self.kdTree = scipy.spatial.KDTree(self.points)
        self.orientation = orientation
        self.tolerance = tolerance

    def uniformPointInner(self):
        i = random.randrange(0, len(self.points))
        return self.orient(Vector(*self.points[i]))

    def intersect(self, other, triedReversed=False):
        def sampler(intRegion):
            o = intRegion.regions[1]
            center, radius = o.circumcircle
            possibles = (Vector(*self.kdTree.data[i])
                         for i in self.kdTree.query_ball_point(center, radius))
            intersection = [p for p in possibles if o.containsPoint(p)]
            if len(intersection) == 0:
                raise RejectionException(f'empty intersection of Regions {self} and {o}')
            return self.orient(random.choice(intersection))
        orientation = orientationFor(self, other, triedReversed)
        return IntersectionRegion(self, other, sampler=sampler, orientation=orientation)

    def containsPoint(self, point):
        point = toVector(point).coordinates
        distance, location = self.kdTree.query(point)
        return (distance <= self.tolerance)

    def containsObject(self, obj):
        return False

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

    def __eq__(self, other):
        if type(other) is not PointSetRegion:
            return NotImplemented
        return (self.name == other.name
                and numpy.array_equal(self.points, other.points)
                and self.orientation == other.orientation)

    @cached
    def __hash__(self):
        return hash((self.name, self.points.tobytes(), self.orientation))

@distributionFunction
def convertToFootprint(region):
    if isinstance(region, PolygonalRegion):
        return region.footprint

    if isinstance(region, IntersectionRegion):
        subregions = [convertToFootprint(r) for r in region.regions]
        return IntersectionRegion(*subregions)

    if isinstance(region, DifferenceRegion):
        return DifferenceRegion(convertToFootprint(region.regionA), convertToFootprint(region.regionB))

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
        return (self.grid[y, x] == 0)

    def containsObject(self, obj):
        # TODO improve this procedure!
        # Fast check
        for c in obj.corners:
            if not self.containsPoint(c):
                return False
        # Slow check
        gps = [self.pointToGrid(corner) for corner in obj.corners]
        x, y = zip(*gps)
        minx, maxx = findMinMax(x)
        miny, maxy = findMinMax(y)
        for x in range(minx, maxx+1):
            for y in range(miny, maxy+1):
                p = self.gridToPoint((x, y))
                if self.grid[y, x] == 1 and obj.containsPoint(p):
                    return False
        return True

###################################################################################################
# View Regions
###################################################################################################

class DefaultViewRegion(MeshVolumeRegion):
    """ The default view region shape.

    The default view region can take several forms, depending on the viewAngles parameter:

    * Case 1:       viewAngles[0] = 360 degrees

      * Case 1.a:   viewAngles[1] = 180 degrees         => Full Sphere View Region
      * Case 1.b:   viewAngles[1] < 180 degrees         => Sphere - (Cone + Cone) (Cones on z axis expanding from origin)

    * Case 2:       viewAngles[0] = 180 degrees

      * Case 2.a:   viewAngles[1] = 180 degrees         => Hemisphere View Region
      * Case 2.b:   viewAngles[1] < 180 degrees         => Hemisphere - (Cone + Cone) (Cones on appropriate hemispheres)

    * Case 3:       viewAngles[1] = 180 degrees

      * Case 3.a:   viewAngles[0] < 180 degrees         => Sphere intersected with Pyramid View Region
      * Case 3.b:   viewAngles[0] > 180 degrees         => Sphere - Backwards Pyramid View Region

    * Case 4:       viewAngles[0 & 1] < 180             => Capped Pyramid View Region
    * Case 5:       viewAngles[0] > 180, Altitude < 180 => (Sphere - (Cone + Cone) (Cones on appropriate hemispheres)) - Backwards Capped Pyramid View Region

    Args:
        visibleDistance: The view distance for this region.
        viewAngles: The view angles for this region.
        name: An optional name to help with debugging.
        position: An optional position, which determines where the center of the region will be.
        rotation: An optional Orientation object which determines the rotation of the object in space.
        orientation: An optional vector field describing the preferred orientation at every point in
          the region.
        tolerance: Tolerance for collision computations.
    """
    def __init__(self, visibleDistance, viewAngles, name=None, position=Vector(0,0,0), rotation=None,\
        orientation=None, tolerance=1e-8):
        # Bound viewAngles from either side.
        viewAngles = tuple([min(angle, math.tau) for angle in viewAngles])

        if min(viewAngles) <= 0:
            raise ValueError("viewAngles cannot have a component less than or equal to 0")

        # Cases in view region computation
        # Case 1:       Azimuth view angle = 360 degrees
        #   Case 1.a:   Altitude view angle = 180 degrees   => Full Sphere View Region
        #   Case 1.b:   Altitude view angle < 180 degrees   => Sphere - (Cone + Cone) (Cones on z axis expanding from origin)
        # Case 2:       Azimuth view angle = 180 degrees
        #   Case 2.a:   Altitude view angle = 180 degrees   => Hemisphere View Region
        #   Case 2.b:   Altitude view angle < 180 degrees   => Hemisphere - (Cone + Cone) (Cones on appropriate hemispheres)
        # Case 3:       Altitude view angle = 180 degrees   
        #   Case 3.a:   Azimuth view angle < 180 degrees    => Sphere intersected with Pyramid View Region
        #   Case 3.b:   Azimuth view angle > 180 degrees    => Sphere - Backwards Pyramid View Region 
        # Case 4:       Both view angles < 180              => Capped Pyramid View Region
        # Case 5:       Azimuth > 180, Altitude < 180       => (Sphere - (Cone + Cone) (Cones on appropriate hemispheres)) - Backwards Capped Pyramid View Region

        view_region = None
        diameter = 2*visibleDistance
        base_sphere = SpheroidRegion(dimensions=(diameter, diameter, diameter), engine="scad")

        if (math.tau-0.01 <= viewAngles[0] <= math.tau+0.01):
            # Case 1
            if viewAngles[1] > math.pi-0.01:
                #Case 1.a
                view_region = base_sphere
            else:
                # Case 1.b
                # Create cone with yaw oriented around (0,0,-1)
                padded_height = visibleDistance * 2
                radius = padded_height*math.tan((math.pi-viewAngles[1])/2)

                cone_mesh = trimesh.creation.cone(radius=radius, height=padded_height)

                position_matrix = translation_matrix((0,0,-1*padded_height))
                cone_mesh.apply_transform(position_matrix)

                # Create two cones around the yaw axis
                orientation_1 = Orientation.fromEuler(0,0,0)
                orientation_2 = Orientation.fromEuler(0,0,math.pi)

                cone_1 = MeshVolumeRegion(mesh=cone_mesh, rotation=orientation_1, center_mesh=False)
                cone_2 = MeshVolumeRegion(mesh=cone_mesh, rotation=orientation_2, center_mesh=False)

                view_region = base_sphere.difference(cone_1).difference(cone_2)

        elif (math.pi-0.01 <= viewAngles[0] <= math.pi+0.01):
            # Case 2
            if viewAngles[1] > math.pi-0.01:
                # Case 2.a
                padded_diameter = 1.1*diameter
                view_region = base_sphere.intersect(BoxRegion(dimensions=(padded_diameter, padded_diameter, padded_diameter), position=(0,padded_diameter/2,0)))
            else:
                # Case 2.b
                # Create cone with yaw oriented around (0,0,-1)
                padded_height = visibleDistance * 2
                radius = padded_height*math.tan((math.pi-viewAngles[1])/2)

                cone_mesh = trimesh.creation.cone(radius=radius, height=padded_height)

                position_matrix = translation_matrix((0,0,-1*padded_height))
                cone_mesh.apply_transform(position_matrix)

                # Create two cones around the yaw axis
                orientation_1 = Orientation.fromEuler(0,0,0)
                orientation_2 = Orientation.fromEuler(0,0,math.pi)

                cone_1 = MeshVolumeRegion(mesh=cone_mesh, rotation=orientation_1, center_mesh=False)
                cone_2 = MeshVolumeRegion(mesh=cone_mesh, rotation=orientation_2, center_mesh=False)

                padded_diameter = 1.1*diameter

                base_hemisphere = base_sphere.intersect(BoxRegion(dimensions=(padded_diameter, padded_diameter, padded_diameter), position=(0,padded_diameter/2,0)))

                view_region = base_hemisphere.difference(cone_1).difference(cone_2)

        elif viewAngles[1] > math.pi-0.01:
            # Case 3
            if viewAngles[0] < math.pi:
                view_region = base_sphere.intersect(TriangularPrismViewRegion(visibleDistance, viewAngles[0]))
            elif viewAngles[0] > math.pi:
                back_tprism = TriangularPrismViewRegion(visibleDistance, math.tau - viewAngles[0], rotation=Orientation.fromEuler(math.pi, 0, 0))
                view_region = base_sphere.difference(back_tprism)
            else:
                assert False, f"{viewAngles=}"

        elif viewAngles[0] < math.pi and viewAngles[1] < math.pi:
            # Case 4
            view_region = base_sphere.intersect(PyramidViewRegion(visibleDistance, viewAngles))
        elif viewAngles[0] > math.pi and viewAngles[1] < math.pi:
            # Case 5
            # Create cone with yaw oriented around (0,0,-1)
            padded_height = visibleDistance * 2
            radius = padded_height*math.tan((math.pi-viewAngles[1])/2)

            cone_mesh = trimesh.creation.cone(radius=radius, height=padded_height)

            position_matrix = translation_matrix((0,0,-1*padded_height))
            cone_mesh.apply_transform(position_matrix)

            # Position on the yaw axis
            orientation_1 = Orientation.fromEuler(0,0,0)
            orientation_2 = Orientation.fromEuler(0,0,math.pi)

            cone_1 = MeshVolumeRegion(mesh=cone_mesh, rotation=orientation_1, center_mesh=False)
            cone_2 = MeshVolumeRegion(mesh=cone_mesh, rotation=orientation_2, center_mesh=False)

            backwards_view_angle = (math.tau-viewAngles[0], math.pi-0.01)
            back_pyramid = PyramidViewRegion(visibleDistance, backwards_view_angle, rotation=Orientation.fromEuler(math.pi, 0, 0))

            # Note: Openscad does not like the result of the difference with the cones, so they must be done last.
            view_region = base_sphere.difference(back_pyramid).difference(cone_1).difference(cone_2)
        else:
            assert False, f"{viewAngles=}"

        assert view_region is not None

        # Initialize volume region
        super().__init__(mesh=view_region.mesh, name=name, position=position, rotation=rotation, orientation=orientation, \
            tolerance=tolerance, center_mesh=False)


class PyramidViewRegion(MeshVolumeRegion):
    """ A pyramid region, used to construct view regions.

    Args:
        visibleDistance: The view distance for this region, equal to the central height of the pyramid 
          (will be slightly amplified to prevent mesh intersection errors).
        viewAngles: The view angles for this region.
        rotation: An optional Orientation object which determines the rotation of the object in space.
    """
    def __init__(self, visibleDistance, viewAngles, rotation=None):
        if min(viewAngles) <= 0 or max(viewAngles) >= math.pi:
            raise ValueError("viewAngles members must be between 0 and Pi.")

        x_dim = 2*visibleDistance*math.tan(viewAngles[0]/2)
        z_dim = 2*visibleDistance*math.tan(viewAngles[1]/2)

        dimensions = (x_dim, visibleDistance*1.01, z_dim)

        # Create pyramid mesh and scale it appropriately.
        vertices = [[ 0,  0,  0],
                    [-1,  1,  1],
                    [ 1,  1,  1],
                    [ 1,  1, -1],
                    [-1,  1, -1]]

        faces = [[0,2,1],
                 [0,3,2],
                 [0,4,3],
                 [0,1,4],
                 [1,2,4],
                 [2,3,4]]

        pyramid_mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

        scale = pyramid_mesh.extents / numpy.array(dimensions)

        scale_matrix = numpy.eye(4)
        scale_matrix[:3, :3] /= scale

        pyramid_mesh.apply_transform(scale_matrix)

        super().__init__(mesh=pyramid_mesh, rotation=rotation, center_mesh=False)


class TriangularPrismViewRegion(MeshVolumeRegion):
    """ A triangular prism region, used to construct view regions.

    Args:
        visibleDistance: The view distance for this region (will be slightly amplified to 
          prevent mesh intersection errors).
        viewAngles: The view angles for this region.
        rotation: An optional Orientation object which determines the rotation of the object in space.
    """
    def __init__(self, visibleDistance, viewAngle, rotation=None):
        if viewAngle <= 0 or viewAngle >= math.pi:
            raise ValueError("viewAngles members must be between 0 and Pi.")

        y_dim = 1.01*visibleDistance
        z_dim = 2*y_dim
        x_dim = 2*math.tan(viewAngle/2)*y_dim

        dimensions = (x_dim, y_dim, z_dim)

        # Create triangualr prism mesh and scale it appropriately.
        vertices = [[ 0,  0,  1], # 0 - Top origin
                    [ 0,  0, -1], # 1 - Bottom origin
                    [-1,  1,  1], # 2 - Top left
                    [ 1,  1,  1], # 3 - Top right
                    [-1,  1, -1], # 4 - Bottom left
                    [ 1,  1, -1]] # 5 - Bottom right

        faces = [
                 [0,3,2], # Top triangle
                 [1,4,5], # Bottom triangle
                 [1,0,2], # Left 1
                 [1,2,4], # Left 2
                 [1,3,0], # Right 1
                 [1,5,3], # Right 2
                 [4,2,3], # Back 1
                 [4,3,5], # Back 2
                ]

        tprism_mesh = trimesh.Trimesh(vertices=vertices, faces=faces)

        scale = tprism_mesh.extents / numpy.array(dimensions)

        scale_matrix = numpy.eye(4)
        scale_matrix[:3, :3] /= scale

        tprism_mesh.apply_transform(scale_matrix)

        super().__init__(mesh=tprism_mesh, rotation=rotation, center_mesh=False)

