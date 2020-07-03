"""Objects representing regions in space."""

import math
import random
import itertools

import numpy
import scipy.spatial
import shapely.geometry
import shapely.ops

from scenic.core.distributions import Samplable, RejectionException, needsSampling
from scenic.core.lazy_eval import valueInContext
from scenic.core.vectors import Vector, OrientedVector, VectorDistribution
from scenic.core.geometry import RotatedRectangle
from scenic.core.geometry import sin, cos, hypot, findMinMax, pointIsInCone, averageVectors
from scenic.core.geometry import headingOfSegment, triangulatePolygon, plotPolygon, polygonUnion
from scenic.core.type_support import toVector
from scenic.core.utils import cached, areEquivalent

def toPolygon(thing):
	if needsSampling(thing):
		return None
	if hasattr(thing, 'polygon'):
		return thing.polygon
	if hasattr(thing, 'polygons'):
		return thing.polygons
	if hasattr(thing, 'lineString'):
		return thing.lineString
	return None

def regionFromShapelyObject(obj, orientation=None):
	"""Build a 'Region' from Shapely geometry."""
	assert obj.is_valid, obj
	if obj.is_empty:
		return nowhere
	elif isinstance(obj, (shapely.geometry.Polygon, shapely.geometry.MultiPolygon)):
		return PolygonalRegion(polygon=obj, orientation=orientation)
	elif isinstance(obj, (shapely.geometry.LineString, shapely.geometry.MultiLineString)):
		return PolylineRegion(polyline=obj, orientation=orientation)
	else:
		raise RuntimeError(f'unhandled type of Shapely geometry: {obj}')

class PointInRegionDistribution(VectorDistribution):
	"""Uniform distribution over points in a Region"""
	def __init__(self, region):
		super().__init__(region)
		self.region = region

	def sampleGiven(self, value):
		return value[self.region].uniformPointInner()

	def __str__(self):
		return f'PointIn({self.region})'

class Region(Samplable):
	"""Abstract class for regions."""
	def __init__(self, name, *dependencies, orientation=None):
		super().__init__(dependencies)
		self.name = name
		self.orientation = orientation

	def sampleGiven(self, value):
		return self

	def intersect(self, other, triedReversed=False):
		"""Get a `Region` representing the intersection of this one with another."""
		if triedReversed:
			return IntersectionRegion(self, other)
		else:
			return other.intersect(self, triedReversed=True)

	def union(self, other, triedReversed=False):
		"""Get a `Region` representing the union of this one with another.

		Not supported by all region types.
		"""
		if triedReversed:
			raise NotImplementedError
		else:
			return other.union(self, triedReversed=True)

	@staticmethod
	def uniformPointIn(region):
		"""Get a uniform `Distribution` over points in a `Region`."""
		return PointInRegionDistribution(region)

	def uniformPoint(self):
		"""Sample a uniformly-random point in this `Region`.

		Can only be called on fixed Regions with no random parameters.
		"""
		assert not needsSampling(self)
		return self.uniformPointInner()

	def uniformPointInner(self):
		"""Do the actual random sampling. Implemented by subclasses."""
		raise NotImplementedError

	def containsPoint(self, point):
		"""Check if the `Region` contains a point. Implemented by subclasses."""
		raise NotImplementedError

	def containsObject(self, obj):
		"""Check if the `Region` contains an :obj:`~scenic.core.object_types.Object`.

		The default implementation assumes the `Region` is convex; subclasses must
		override the method if this is not the case.
		"""
		for corner in obj.corners:
			if not self.containsPoint(corner):
				return False
		return True

	def __contains__(self, thing):
		"""Check if this `Region` contains an object or vector."""
		from scenic.core.object_types import Object
		if isinstance(thing, Object):
			return self.containsObject(thing)
		vec = toVector(thing, '"X in Y" with X not an Object or a vector')
		return self.containsPoint(vec)

	def getAABB(self):
		"""Axis-aligned bounding box for this `Region`. Implemented by some subclasses."""
		raise NotImplementedError

	def orient(self, vec):
		"""Orient the given vector along the region's orientation, if any."""
		if self.orientation is None:
			return vec
		else:
			return OrientedVector(vec.x, vec.y, self.orientation[vec])

	def __str__(self):
		return f'<Region {self.name}>'

class AllRegion(Region):
	"""Region consisting of all space."""
	def intersect(self, other, triedReversed=False):
		return other

	def containsPoint(self, point):
		return True

	def containsObject(self, obj):
		return True

	def __eq__(self, other):
		return type(other) is AllRegion

	def __hash__(self):
		return hash(AllRegion)

class EmptyRegion(Region):
	"""Region containing no points."""
	def intersect(self, other, triedReversed=False):
		return self

	def union(self, other, triedReversed=False):
		return other

	def uniformPointInner(self):
		raise RejectionException(f'sampling empty Region')

	def containsPoint(self, point):
		return False

	def containsObject(self, obj):
		return False

	def show(self, plt, style=None):
		pass

	def __eq__(self, other):
		return type(other) is EmptyRegion

	def __hash__(self):
		return hash(EmptyRegion)

everywhere = AllRegion('everywhere')
nowhere = EmptyRegion('nowhere')

class CircularRegion(Region):
	def __init__(self, center, radius, resolution=32):
		super().__init__('Circle', center, radius)
		self.center = center.toVector()
		self.radius = radius
		self.circumcircle = (self.center, self.radius)

		if not (needsSampling(self.center) or needsSampling(self.radius)):
			ctr = shapely.geometry.Point(self.center)
			self.polygon = ctr.buffer(self.radius, resolution=resolution)

	def sampleGiven(self, value):
		return CircularRegion(value[self.center], value[self.radius])

	def evaluateInner(self, context):
		center = valueInContext(self.center, context)
		radius = valueInContext(self.radius, context)
		return CircularRegion(center, radius)

	def containsPoint(self, point):
		point = point.toVector()
		return point.distanceTo(self.center) <= self.radius

	def uniformPointInner(self):
		x, y = self.center
		r = random.triangular(0, self.radius, self.radius)
		t = random.uniform(-math.pi, math.pi)
		pt = Vector(x + (r * cos(t)), y + (r * sin(t)))
		return self.orient(pt)

	def getAABB(self):
		x, y = self.center
		r = self.radius
		return ((x - r, y - r), (x + r, y + r))

	def isEquivalentTo(self, other):
		if type(other) is not CircularRegion:
			return False
		return (areEquivalent(other.center, self.center)
		        and areEquivalent(other.radius, self.radius))

	def __str__(self):
		return f'CircularRegion({self.center}, {self.radius})'

class SectorRegion(Region):
	def __init__(self, center, radius, heading, angle, resolution=32):
		super().__init__('Sector', center, radius, heading, angle)
		self.center = center.toVector()
		self.radius = radius
		self.heading = heading
		self.angle = angle
		r = (radius / 2) * cos(angle / 2)
		self.circumcircle = (self.center.offsetRadially(r, heading), r)

		if not any(needsSampling(x) for x in (self.center, radius, heading, angle)):
			ctr = shapely.geometry.Point(self.center)
			circle = ctr.buffer(self.radius, resolution=resolution)
			if angle >= math.tau - 0.001:
				self.polygon = circle
			else:
				mask = shapely.geometry.Polygon([
				    self.center,
				    self.center.offsetRadially(radius, heading + angle/2),
				    self.center.offsetRadially(2*radius, heading),
				    self.center.offsetRadially(radius, heading - angle/2)
				])
				self.polygon = circle & mask

	def sampleGiven(self, value):
		return SectorRegion(value[self.center], value[self.radius],
			value[self.heading], value[self.angle])

	def evaluateInner(self, context):
		center = valueInContext(self.center, context)
		radius = valueInContext(self.radius, context)
		heading = valueInContext(self.heading, context)
		angle = valueInContext(self.angle, context)
		return SectorRegion(center, radius, heading, angle)

	def containsPoint(self, point):
		point = point.toVector()
		if not pointIsInCone(tuple(point), tuple(self.center), self.heading, self.angle):
			return False
		return point.distanceTo(self.center) <= self.radius

	def uniformPointInner(self):
		x, y = self.center
		heading, angle, maxDist = self.heading, self.angle, self.radius
		r = random.triangular(0, maxDist, maxDist)
		ha = angle / 2.0
		t = random.uniform(-ha, ha) + (heading + (math.pi / 2))
		pt = Vector(x + (r * cos(t)), y + (r * sin(t)))
		return self.orient(pt)

	def isEquivalentTo(self, other):
		if type(other) is not SectorRegion:
			return False
		return (areEquivalent(other.center, self.center)
		        and areEquivalent(other.radius, self.radius)
		        and areEquivalent(other.heading, self.heading)
		        and areEquivalent(other.angle, self.angle))

	def __str__(self):
		return f'SectorRegion({self.center},{self.radius},{self.heading},{self.angle})'

class RectangularRegion(RotatedRectangle, Region):
	def __init__(self, position, heading, width, height):
		super().__init__('Rectangle', position, heading, width, height)
		self.position = position.toVector()
		self.heading = heading
		self.width = width
		self.height = height
		self.hw = hw = width / 2
		self.hh = hh = height / 2
		self.radius = hypot(hw, hh)		# circumcircle; for collision detection
		self.corners = tuple(position.offsetRotated(heading, Vector(*offset))
			for offset in ((hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh)))
		self.circumcircle = (self.position, self.radius)

	def sampleGiven(self, value):
		return RectangularRegion(value[self.position], value[self.heading],
			value[self.width], value[self.height])

	def evaluateInner(self, context):
		position = valueInContext(self.position, context)
		heading = valueInContext(self.heading, context)
		width = valueInContext(self.width, context)
		height = valueInContext(self.height, context)
		return RectangularRegion(position, heading, width, height)

	def uniformPointInner(self):
		hw, hh = self.hw, self.hh
		rx = random.uniform(-hw, hw)
		ry = random.uniform(-hh, hh)
		pt = self.position.offsetRotated(self.heading, Vector(rx, ry))
		return self.orient(pt)

	def getAABB(self):
		x, y = zip(*self.corners)
		minx, maxx = findMinMax(x)
		miny, maxy = findMinMax(y)
		return ((minx, miny), (maxx, maxy))

	def isEquivalentTo(self, other):
		if type(other) is not RectangularRegion:
			return False
		return (areEquivalent(other.position, self.position)
		        and areEquivalent(other.heading, self.heading)
		        and areEquivalent(other.width, self.width)
		        and areEquivalent(other.height, self.height))

	def __str__(self):
		return f'RectangularRegion({self.position},{self.heading},{self.width},{self.height})'

class PolylineRegion(Region):
	"""Region given by one or more polylines (chain of line segments)"""
	def __init__(self, points=None, polyline=None, orientation=True):
		super().__init__('Polyline', orientation=orientation)
		if points is not None:
			points = tuple(points)
			if len(points) < 2:
				raise RuntimeError('tried to create PolylineRegion with < 2 points')
			self.points = points
			self.lineString = shapely.geometry.LineString(points)
		elif polyline is not None:
			if isinstance(polyline, shapely.geometry.LineString):
				if len(polyline.coords) < 2:
					raise RuntimeError('tried to create PolylineRegion with <2-point LineString')
			elif isinstance(polyline, shapely.geometry.MultiLineString):
				if len(polyline) == 0:
					raise RuntimeError('tried to create PolylineRegion from empty MultiLineString')
				for line in polyline:
					assert len(line.coords) >= 2
			else:
				raise RuntimeError('tried to create PolylineRegion from non-LineString')
			self.lineString = polyline
			self.points = None
		else:
			raise RuntimeError('must specify points or polyline for PolylineRegion')
		if not self.lineString.is_valid:
			raise RuntimeError('tried to create PolylineRegion with '
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
			for p, q in self.segments:
				pts.append(p)
			pts.append(q)
			self.points = pts

	@classmethod
	def segmentsOf(cls, lineString):
		if isinstance(lineString, shapely.geometry.LineString):
			segments = []
			points = list(lineString.coords)
			if len(points) < 2:
				raise RuntimeError('LineString has fewer than 2 points')
			last = points[0]
			for point in points[1:]:
				segments.append((last, point))
				last = point
			return segments
		elif isinstance(lineString, shapely.geometry.MultiLineString):
			allSegments = []
			for line in lineString:
				allSegments.extend(cls.segmentsOf(line))
			return allSegments
		else:
			raise RuntimeError('called segmentsOf on non-linestring')

	def uniformPointInner(self):
		pointA, pointB = random.choices(self.segments,
		                                cum_weights=self.cumulativeLengths)[0]
		interpolation = random.random()
		x, y = averageVectors(pointA, pointB, weight=interpolation)
		if self.orientation is True:
			return OrientedVector(x, y, headingOfSegment(pointA, pointB))
		else:
			return self.orient(Vector(x, y))

	def intersect(self, other, triedReversed=False):
		poly = toPolygon(other)
		if poly is not None:
			intersection = self.lineString & poly
			if (intersection.is_empty or
			    not isinstance(intersection, (shapely.geometry.LineString,
			                                  shapely.geometry.MultiLineString))):
				# TODO handle points!
				return nowhere
			return PolylineRegion(polyline=intersection)
		return super().intersect(other, triedReversed)

	def containsPoint(self, point):
		return self.lineString.intersects(shapely.geometry.Point(point))

	def containsObject(self, obj):
		return False

	def getAABB(self):
		xmin, ymin, xmax, ymax = self.lineString.bounds
		return ((xmin, ymin), (xmax, ymax))

	def show(self, plt, style='r-'):
		for pointA, pointB in self.segments:
			plt.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], style)

	def __getitem__(self, i):
		"""Get the ith point along this polyline.

		If the region consists of multiple polylines, this order is linear
		along each polyline but arbitrary across different polylines.
		"""
		return Vector(*self.points[i])

	def __str__(self):
		return f'PolylineRegion({self.lineString})'

	def __eq__(self, other):
		if type(other) is not PolylineRegion:
			return NotImplemented
		return (other.lineString == self.lineString)

	@cached
	def __hash__(self):
		return hash(str(self.lineString))

class PolygonalRegion(Region):
	"""Region given by one or more polygons (possibly with holes)"""
	def __init__(self, points=None, polygon=None, orientation=None):
		super().__init__('Polygon', orientation=orientation)
		if polygon is None and points is None:
			raise RuntimeError('must specify points or polygon for PolygonalRegion')
		if polygon is None:
			points = tuple(points)
			if len(points) == 0:
				raise RuntimeError('tried to create PolygonalRegion from empty point list!')
			for point in points:
				if needsSampling(point):
					raise RuntimeError('only fixed PolygonalRegions are supported')
			self.points = points
			polygon = shapely.geometry.Polygon(points)

		if isinstance(polygon, shapely.geometry.Polygon):
			self.polygons = shapely.geometry.MultiPolygon([polygon])
		elif isinstance(polygon, shapely.geometry.MultiPolygon):
			self.polygons = polygon
		else:
			raise RuntimeError(f'tried to create PolygonalRegion from non-polygon {polygon}')
		if not self.polygons.is_valid:
			raise RuntimeError('tried to create PolygonalRegion with '
			                   f'invalid polygon {self.polygons}')

		if points is None and len(self.polygons) == 1 and len(self.polygons[0].interiors) == 0:
			self.points = tuple(self.polygons[0].exterior.coords[:-1])

		if self.polygons.is_empty:
			raise RuntimeError('tried to create empty PolygonalRegion')

		triangles = []
		for polygon in self.polygons:
			triangles.extend(triangulatePolygon(polygon))
		assert len(triangles) > 0, self.polygons
		self.trianglesAndBounds = tuple((tri, tri.bounds) for tri in triangles)
		areas = (triangle.area for triangle in triangles)
		self.cumulativeTriangleAreas = tuple(itertools.accumulate(areas))

	def uniformPointInner(self):
		triangle, bounds = random.choices(
			self.trianglesAndBounds,
			cum_weights=self.cumulativeTriangleAreas)[0]
		minx, miny, maxx, maxy = bounds
		# TODO improve?
		while True:
			x, y = random.uniform(minx, maxx), random.uniform(miny, maxy)
			if triangle.intersects(shapely.geometry.Point(x, y)):
				return self.orient(Vector(x, y))

	def intersect(self, other, triedReversed=False):
		poly = toPolygon(other)
		orientation = other.orientation if self.orientation is None else self.orientation
		if poly is not None:
			intersection = self.polygons & poly
			if intersection.is_empty:
				return nowhere
			elif isinstance(intersection, (shapely.geometry.Polygon,
			                             shapely.geometry.MultiPolygon)):
				return PolygonalRegion(polygon=intersection, orientation=orientation)
			elif isinstance(intersection, shapely.geometry.GeometryCollection):
				polys = []
				for geom in intersection:
					if isinstance(geom, shapely.geometry.Polygon):
						polys.append(geom)
				if len(polys) == 0:
					# TODO handle points, lines
					raise RuntimeError('unhandled type of polygon intersection')
				intersection = shapely.geometry.MultiPolygon(polys)
				return PolygonalRegion(polygon=intersection, orientation=orientation)
			else:
				# TODO handle points, lines
				raise RuntimeError('unhandled type of polygon intersection')
		return super().intersect(other, triedReversed)

	def union(self, other, triedReversed=False, buf=0):
		poly = toPolygon(other)
		if not poly:
			return super().union(other, triedReversed)
		union = polygonUnion((self.polygons, poly), buf=buf)
		return PolygonalRegion(polygon=union)

	@staticmethod
	def unionAll(regions, buf=0):
		polys = []
		for reg in regions:
			if reg != nowhere:
				polys.append(toPolygon(reg))
		if not polys:
			return nowhere
		if any(not poly for poly in polys):
			raise RuntimeError(f'cannot take union of regions {regions}')
		union = polygonUnion(polys, buf=buf)
		return PolygonalRegion(polygon=union)

	def containsPoint(self, point):
		return self.polygons.intersects(shapely.geometry.Point(point))

	def containsObject(self, obj):
		objPoly = obj.polygon
		if objPoly is None:
			raise RuntimeError('tried to test containment of symbolic Object!')
		# TODO improve boundary handling?
		return self.polygons.contains(objPoly)

	def getAABB(self):
		xmin, xmax, ymin, ymax = self.polygons.bounds
		return ((xmin, ymin), (xmax, ymax))

	def show(self, plt, style='r-'):
		plotPolygon(self.polygons, plt, style=style)

	def __str__(self):
		return '<PolygonalRegion>'

	def __eq__(self, other):
		if type(other) is not PolygonalRegion:
			return NotImplemented
		return (other.polygons == self.polygons
		        and other.orientation == self.orientation)

	@cached
	def __hash__(self):
		# TODO better way to hash mutable Shapely geometries? (also for PolylineRegion)
		return hash((str(self.polygons), self.orientation))

class PointSetRegion(Region):
	"""Region consisting of a set of discrete points.

	No :obj:`~scenic.core.object_types.Object` can be contained in a `PointSetRegion`,
	since the latter is discrete. (This may not be true for subclasses, e.g.
	`GridRegion`.)

	Args:
		name (str): name for debugging
		points (iterable): set of points comprising the region
		kdtree (:obj:`scipy.spatial.KDTree`, optional): k-D tree for the points (one will
		  be computed if none is provided)
		orientation (:obj:`~scenic.core.vectors.VectorField`, optional): orientation for
		  the region
		tolerance (float, optional): distance tolerance for checking whether a point lies
		  in the region
	"""

	def __init__(self, name, points, kdTree=None, orientation=None, tolerance=1e-6):
		super().__init__(name, orientation=orientation)
		self.points = tuple(points)
		for point in self.points:
			if needsSampling(point):
				raise RuntimeError('only fixed PointSetRegions are supported')
		self.kdTree = scipy.spatial.cKDTree(self.points) if kdTree is None else kdTree
		self.orientation = orientation
		self.tolerance = tolerance

	def uniformPointInner(self):
		return self.orient(Vector(*random.choice(self.points)))

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
		return IntersectionRegion(self, other, sampler=sampler, orientation=self.orientation)

	def containsPoint(self, point):
		distance, location = self.kdTree.query(point)
		return (distance <= self.tolerance)

	def containsObject(self, obj):
		raise NotImplementedError()

	def __eq__(self, other):
		if type(other) is not PointSetRegion:
			return NotImplemented
		return (other.name == self.name
		        and other.points == self.points
		        and other.orientation == self.orientation)

	def __hash__(self):
		return hash((self.name, self.points, self.orientation))

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
		orientation (:obj:`~scenic.core.vectors.VectorField`, optional): orientation of region
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
		x, y = point
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

class IntersectionRegion(Region):
	def __init__(self, *regions, orientation=None, sampler=None):
		self.regions = tuple(regions)
		if len(self.regions) < 2:
			raise RuntimeError('tried to take intersection of fewer than 2 regions')
		super().__init__('Intersection', *self.regions, orientation=orientation)
		if sampler is None:
			sampler = self.genericSampler
		self.sampler = sampler

	def sampleGiven(self, value):
		regs = [value[reg] for reg in self.regions]
		# Now that regions have been sampled, attempt intersection again in the hopes
		# there is a specialized sampler to handle it (unless we already have one)
		if self.sampler is self.genericSampler:
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
		                          sampler=self.sampler)

	def evaluateInner(self, context):
		regs = (valueInContext(reg, context) for reg in self.regions)
		orientation = valueInContext(self.orientation, context)
		return IntersectionRegion(*regs, orientation=orientation, sampler=self.sampler)

	def containsPoint(self, point):
		return all(region.containsPoint(point) for region in self.regions)

	def uniformPointInner(self):
		return self.orient(self.sampler(self))

	@staticmethod
	def genericSampler(intersection):
		regs = intersection.regions
		point = regs[0].uniformPointInner()
		for region in regs[1:]:
			if not region.containsPoint(point):
				raise RejectionException(
				    f'sampling intersection of Regions {regs[0]} and {region}')
		return point

	def isEquivalentTo(self, other):
		if type(other) is not IntersectionRegion:
			return False
		return (areEquivalent(set(other.regions), set(self.regions))
		        and other.orientation == self.orientation)

	def __str__(self):
		return f'IntersectionRegion({self.regions})'
