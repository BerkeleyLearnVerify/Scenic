
### Regions

import math
import random
import itertools

import numpy
import scipy.spatial
import shapely.geometry
import shapely.ops

from scenic.core.distributions import Samplable, RejectionException, needsSampling
from scenic.core.vectors import Vector, OrientedVector, VectorDistribution
from scenic.core.geometry import RotatedRectangle
from scenic.core.geometry import sin, cos, hypot, findMinMax, pointIsInCone, averageVectors
from scenic.core.geometry import headingOfSegment, triangulatePolygon, plotPolygon, polygonUnion

def toPolygon(thing):
	if hasattr(thing, 'polygon'):
		return thing.polygon
	if hasattr(thing, 'polygons'):
		return thing.polygons
	return None

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
	def __init__(self, name, *dependencies, orientation=None):
		super().__init__(dependencies)
		self.name = name
		self.orientation = orientation

	def sampleGiven(self, value):
		return self

	def intersect(self, other, triedReversed=False):
		if triedReversed:
			return IntersectionRegion(self, other)
		else:
			return other.intersect(self, triedReversed=True)

	@staticmethod
	def uniformPointIn(region):
		return PointInRegionDistribution(region)

	def uniformPoint(self):
		assert not needsSampling(self)
		return self.uniformPointInner()

	def uniformPointInner(self):
		raise NotImplementedError()

	def containsPoint(self, point):
		raise NotImplementedError()

	def containsObject(self, obj):
		# will need to be overridden for non-convex regions
		for corner in obj.corners:
			if not self.containsPoint(corner):
				return False
		return True

	def getAABB(self):
		"""Axis-aligned bounding box for this Region"""
		raise NotImplementedError()

	def orient(self, vec):
		if self.orientation is None:
			return vec
		else:
			return OrientedVector(vec.x, vec.y, self.orientation[vec])

	def __str__(self):
		return f'<Region {self.name}>'

class AllRegion(Region):
	"""Region consisting of all space"""
	def intersect(self, other, triedReversed=False):
		return other

	def containsPoint(self, point):
		return True

	def containsObject(self, obj):
		return True

class EmptyRegion(Region):
	def intersect(self, other, triedReversed=False):
		return self

	def containsPoint(self, point):
		return False

	def containsObject(self, obj):
		return False

everywhere = AllRegion('everywhere')
nowhere = EmptyRegion('nowhere')

class CircularRegion(Region):
	def __init__(self, center, radius):
		super().__init__('Circle', center, radius)
		self.center = center.toVector()
		self.radius = radius
		self.circumcircle = (self.center, self.radius)

	def sampleGiven(self, value):
		return CircularRegion(value[self.center], value[self.radius])

	def containsPoint(self, point):
		point = point.toVector()
		return point.distanceTo(self.center) <= self.radius

	def uniformPointInner(self):
		x, y = self.center
		r = random.triangular(0, self.radius, self.radius)
		t = random.uniform(-math.pi, math.pi)
		return Vector(x + (r * cos(t)), y + (r * sin(t)))

	def getAABB(self):
		x, y = self.center
		r = self.radius
		return ((x - r, y - r), (x + r, y + r))

	def __str__(self):
		return f'CircularRegion({self.center}, {self.radius})'

class SectorRegion(Region):
	def __init__(self, center, radius, heading, angle):
		super().__init__('Sector', center, radius, heading, angle)
		self.center = center.toVector()
		self.radius = radius
		self.heading = heading
		self.angle = angle
		r = (radius / 2) * cos(angle / 2)
		self.circumcircle = (self.center.offsetRadially(r, heading), r)

	def sampleGiven(self, value):
		return SectorRegion(value[self.center], value[self.radius],
			value[self.heading], value[self.angle])

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
		return Vector(x + (r * cos(t)), y + (r * sin(t)))

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

	def uniformPointInner(self):
		hw, hh = self.hw, self.hh
		rx = random.uniform(-hw, hw)
		ry = random.uniform(-hh, hh)
		return self.position.offsetRotated(self.heading, Vector(rx, ry))

	def getAABB(self):
		x, y = zip(*self.corners)
		minx, maxx = findMinMax(x)
		miny, maxy = findMinMax(y)
		return ((minx, miny), (maxx, maxy))

	def __str__(self):
		return f'RectangularRegion({self.position},{self.heading},{self.width},{self.height})'

class PolylineRegion(Region):
	"""Region given by a polyline (chain of line segments)"""
	def __init__(self, points):
		super().__init__('Polyline', orientation=True)
		points = tuple(points)
		if len(points) < 2:
			raise RuntimeError('tried to create PolylineRegion with < 2 points')
		self.points = points
		cumulativeLengths = []
		total = 0
		last = points[0]
		segments = []
		for point in points[1:]:
			segments.append((last, point))
			dx, dy = point[0] - last[0], point[1] - last[1]
			total += math.hypot(dx, dy)
			cumulativeLengths.append(total)
			last = point
		self.cumulativeLengths = cumulativeLengths
		self.segments = segments
		self.lineString = shapely.geometry.LineString(points)
		if not self.lineString.is_valid:
			raise RuntimeError('tried to create PolylineRegion with '
			                   f'invalid LineString {self.lineString}')

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
			if not isinstance(intersection, shapely.geometry.LineString):
				# TODO handle points!
				return nowhere
			return PolylineRegion(tuple(intersection.coords))
		return super().intersect(other, triedReversed)

	def containsPoint(self, point):
		return self.lineString.intersects(shapely.geometry.Point(point))

	def containsObject(self, obj):
		return False

	def getAABB(self):
		xmin, xmax = findMinMax(p[0] for p in self.points)
		ymin, ymax = findMinMax(p[1] for p in self.points)
		return ((xmin, ymin), (xmax, ymax))

	def show(self, plt, style='r-'):
		for pointA, pointB in self.segments:
			plt.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], style)

	def __str__(self):
		return f'PolylineRegion({self.points})'

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

		triangles = []
		for polygon in self.polygons:
			triangles.extend(triangulatePolygon(polygon))
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
			if isinstance(intersection, (shapely.geometry.Polygon,
			                             shapely.geometry.MultiPolygon)):
				return PolygonalRegion(polygon=intersection, orientation=orientation)
			elif intersection.is_empty:
				return nowhere
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

	def union(self, other):
		poly = toPolygon(other)
		if not poly:
			raise RuntimeError(f'cannot take union of PolygonalRegion with {other}')
		union = polygonUnion((self.polygons, poly))
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

class PointSetRegion(Region):
	def __init__(self, name, points, kdTree=None, orientation=None, tolerance=1e-6):
		super().__init__(name, orientation=orientation)
		self.points = list(points)
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
			possibles = (Vector(*self.kdTree.data[i]) for i in self.kdTree.query_ball_point(center, radius))
			intersection = [p for p in possibles if o.containsPoint(p)]
			if len(intersection) == 0:
				raise RejectionException()
			return self.orient(random.choice(intersection))
		return IntersectionRegion(self, other, sampler=sampler, orientation=self.orientation)

	def containsPoint(self, point):
		distance, location = self.kdTree.query(point)
		return (distance <= self.tolerance)

	def containsObject(self, obj):
		raise NotImplementedError()

class GridRegion(PointSetRegion):
	"""A Region given by an obstacle grid"""
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
		self.regions = list(regions)
		if len(self.regions) < 2:
			raise RuntimeError('tried to take intersection of fewer than 2 regions')
		super().__init__('Intersection', *self.regions, orientation=orientation)
		if sampler is None:
			sampler = self.genericSampler
		self.sampler = sampler

	def sampleGiven(self, value):
		regs = (value[reg] for reg in self.regions)
		return IntersectionRegion(*regs, orientation=value[self.orientation],
		                          sampler=self.sampler)

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
				raise RejectionException()
		return point

	def __str__(self):
		return f'IntersectionRegion({self.regions})'
