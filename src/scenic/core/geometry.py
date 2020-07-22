"""Utility functions for geometric computation."""

import math
import itertools
import warnings

import numpy as np
import shapely.geometry
import shapely.ops
import pypoly2tri

from scenic.core.distributions import (needsSampling, distributionFunction,
                                       monotonicDistributionFunction)
from scenic.core.lazy_eval import needsLazyEvaluation
import scenic.core.utils as utils

@distributionFunction
def sin(x) -> float:
	return math.sin(x)

@distributionFunction
def cos(x) -> float:
	return math.cos(x)

@monotonicDistributionFunction
def hypot(x, y) -> float:
	return math.hypot(x, y)

@monotonicDistributionFunction
def max(*args):
	return __builtins__['max'](*args)

@monotonicDistributionFunction
def min(*args):
	return __builtins__['min'](*args)

def normalizeAngle(angle):
	while angle > math.pi:
		angle -= math.tau
	while angle < -math.pi:
		angle += math.tau
	assert -math.pi <= angle <= math.pi
	return angle

def addVectors(a, b):
	ax, ay = a[0], a[1]
	bx, by = b[0], b[1]
	return (ax + bx, ay + by)

def subtractVectors(a, b):
	ax, ay = a[0], a[1]
	bx, by = b[0], b[1]
	return (ax - bx, ay - by)

def averageVectors(a, b, weight=0.5):
	ax, ay = a[0], a[1]
	bx, by = b[0], b[1]
	aw, bw = 1.0 - weight, weight
	return (ax * aw + bx * bw, ay * aw + by * bw)

def rotateVector(vector, angle):
	x, y = vector
	c, s = cos(angle), sin(angle)
	return ((c * x) - (s * y), (s * x) + (c * y))

def findMinMax(iterable):
	minv = float('inf')
	maxv = float('-inf')
	for val in iterable:
		if val < minv:
			minv = val
		if val > maxv:
			maxv = val
	return (minv, maxv)

def radialToCartesian(point, radius, heading):
	angle = heading + (math.pi / 2.0)
	rx, ry = radius * cos(angle), radius * sin(angle)
	return (point[0] + rx, point[1] + ry)

def positionRelativeToPoint(point, heading, offset):
	ro = rotateVector(offset, heading)
	return addVectors(point, ro)

def headingOfSegment(pointA, pointB):
	ax, ay = pointA
	bx, by = pointB
	return normalizeAngle(math.atan2(by - ay, bx - ax) - (math.pi / 2.0))

def viewAngleToPoint(point, base, heading):
	x, y = base
	ox, oy = point
	a = math.atan2(oy - y, ox - x) - (heading + (math.pi / 2.0))
	return normalizeAngle(a)

def apparentHeadingAtPoint(point, heading, base):
	x, y = base
	ox, oy = point
	a = (heading + (math.pi / 2.0)) - math.atan2(oy - y, ox - x)
	return normalizeAngle(a)

def circumcircleOfAnnulus(center, heading, angle, minDist, maxDist):
	m = (minDist + maxDist) / 2.0
	g = (maxDist - minDist) / 2.0
	h = m * math.sin(angle / 2.0)
	h2 = h * h
	d = math.sqrt(h2 + (m * m))
	r = math.sqrt(h2 + (g * g))
	return radialToCartesian(center, d, heading), r

def pointIsInCone(point, base, heading, angle):
	va = viewAngleToPoint(point, base, heading)
	return (abs(va) <= angle / 2.0)

def distanceToLine(point, a, b):
	lx, ly = b[0] - a[0], b[1] - a[1]
	norm = math.hypot(lx, ly)
	nx, ny = -ly/norm, lx/norm
	px, py = point[0] - a[0], point[1] - a[1]
	return abs((px * nx) + (py * ny))

def distanceToSegment(point, a, b):
	lx, ly = b[0] - a[0], b[1] - a[1]
	px, py = point[0] - a[0], point[1] - a[1]
	proj = (px * lx) + (py * ly)
	if proj < 0:
		return math.hypot(px, py)
	lnorm = math.hypot(lx, ly)
	if proj > lnorm * lnorm:
		return math.hypot(px - lx, py - ly)
	else:
		return abs((py * lx) - (px * ly)) / lnorm

def polygonUnion(polys, buf=0, tolerance=0, holeTolerance=0.002):
	if not polys:
		return shapely.geometry.Polygon()
	polys = list(polys)
	if len(polys) == 1:
		return polys[0]
	buffered = [poly.buffer(buf) for poly in polys]
	union = shapely.ops.unary_union(buffered).buffer(-buf)
	assert union.is_valid, union
	if tolerance > 0:
		union = cleanPolygon(union, tolerance, holeTolerance)
		assert union.is_valid, union
		#checkPolygon(union, tolerance)
	return union

def checkPolygon(poly, tolerance):
	def checkPolyline(pl):
		for i, p in enumerate(pl[:-1]):
			q = pl[i+1]
			dx, dy = q[0] - p[0], q[1] - p[1]
			assert math.hypot(dx, dy) >= tolerance
	if isinstance(poly, shapely.geometry.MultiPolygon):
		for p in poly:
			checkPolygon(p, tolerance)
	else:
		checkPolyline(poly.exterior.coords)
		for i in poly.interiors:
			checkPolyline(i.coords)

def cleanPolygon(poly, tolerance, holeTolerance=0, minRelArea=0.05, minHullLenRatio=0.9):
	if holeTolerance == 0:
		holeTolerance = tolerance * tolerance
	if poly.is_empty:
		return poly
	elif isinstance(poly, shapely.geometry.MultiPolygon):
		polys = [cleanPolygon(p, tolerance, holeTolerance) for p in poly]
		total = sum(poly.area for poly in polys)
		kept = []
		for poly in polys:
			area = poly.area
			if area >= holeTolerance or area >= minRelArea * total:
				kept.append(poly)
		poly = shapely.ops.unary_union(kept)
		assert poly.is_valid, poly
		return poly
	exterior = poly.exterior.simplify(tolerance)
	newExterior = cleanChain(exterior.coords, tolerance)
	if len(newExterior) <= 3:
		# attempt to save very thin polygons that would get reduced to a single point
		hull = exterior.convex_hull
		if hull.length >= minHullLenRatio * exterior.length:
			return hull
		return shapely.geometry.Polygon()
	ints = []
	for interior in poly.interiors:
		interior = interior.simplify(tolerance)
		interior = cleanChain(interior.coords, tolerance)
		if len(interior) >= 4:
			hole = shapely.geometry.Polygon(interior)
			if hole.area > holeTolerance:
				ints.append(interior)
	newPoly = shapely.geometry.Polygon(newExterior, ints)
	if not newPoly.is_valid:
		# last-ditch attempt to salvage polygon by splitting across self-intersections
		ext = splitSelfIntersections(newPoly.exterior, minArea=holeTolerance,
		                             minRelArea=minRelArea, minHullLenRatio=minHullLenRatio)
		holes = [
			splitSelfIntersections(hole, minArea=holeTolerance,
			                       minRelArea=minRelArea, minHullLenRatio=minHullLenRatio)
			for hole in newPoly.interiors
		]
		newPoly = ext.difference(shapely.ops.unary_union(holes))
	assert newPoly.is_valid, newPoly
	return newPoly

def splitSelfIntersections(chain, minArea, minRelArea, minHullLenRatio):
	ls = shapely.geometry.LineString(chain)
	parts = list(shapely.ops.polygonize(shapely.ops.unary_union(ls)))
	total = sum(part.area for part in parts)
	if total < minArea:
		# attempt to save very thin polygons that could get thrown out by polygonize
		hull = ls.convex_hull
		if hull.length >= minHullLenRatio * ls.length:
			return hull
	kept = [part for part in parts if part.area >= minArea or (part.area / total) >= minRelArea]
	return shapely.ops.unary_union(kept)

def cleanChain(chain, tolerance=1e-6, lineTolerance=1e-6):
	closed = (tuple(chain[0]) == tuple(chain[-1]))
	minLength = 4 if closed else 3
	if len(chain) <= minLength:
		return chain
	tol2 = tolerance * tolerance
	# collapse nearby points (since Shapely's simplify method doesn't always do this)
	a = chain[0]
	newChain = [a]
	for b in chain[1:]:
		dx, dy = b[0] - a[0], b[1] - a[1]
		if (dx * dx) + (dy * dy) > tol2:
			newChain.append(b)
			a = b
	if closed:
		b = chain[0]
		dx, dy = b[0] - a[0], b[1] - a[1]
		if (dx * dx) + (dy * dy) <= tol2:
			newChain.pop()
		newChain.append(chain[0])
	if len(newChain) <= minLength:
		if not closed:
			newChain.append(chain[-1])
		return newChain
	# collapse collinear points
	chain = newChain
	if closed:
		a = chain[-2]
		b = chain[0]
		ci = 1
		newChain = []
	else:
		a = chain[0]
		b = chain[1]
		ci = 2
		newChain = [a]
	for c in chain[ci:]:
		dx, dy = c[0] - a[0], c[1] - a[1]
		if dx == dy == 0 or distanceToLine(b, a, c) > lineTolerance:
			newChain.append(b)
			a = b
			b = c
		else:
			b = c
	newChain.append(c)
	return newChain

#: Whether to warn when falling back to pypoly2tri for triangulation
givePP2TWarning = True

class TriangulationError(RuntimeError):
	"""Signals that the installed triangulation libraries are insufficient.

	Specifically, raised when pypoly2tri hits the recursion limit trying to
	triangulate a large polygon.
	"""
	pass

def triangulatePolygon(polygon):
	"""Triangulate the given Shapely polygon.

	Note that we can't use ``shapely.ops.triangulate`` since it triangulates
	point sets, not polygons (i.e., it doesn't respect edges). We need an
	algorithm for triangulation of polygons with holes (it doesn't need to be a
	Delaunay triangulation).

	We currently use the GPC library (wrapped by the ``Polygon3`` package) if it is
	installed. Since it is not free for commercial use, we don't require it as a
	dependency, falling back on the BSD-compatible ``pypoly2tri`` as needed. In this
	case we issue a warning, since GPC is more robust and handles large polygons.
	The warning can be disabled by setting `givePP2TWarning` to ``False``.

	Args:
		polygon (shapely.geometry.Polygon): Polygon to triangulate.

	Returns:
		A list of disjoint (except for edges) triangles whose union is the
		original polygon.
	"""
	try:
		import Polygon
		return triangulatePolygon_gpc(polygon)
	except ImportError:
		pass
	if givePP2TWarning:
		warnings.warn('Using pypoly2tri for triangulation; for non-commercial use, consider'
		              ' installing the faster Polygon3 library (pip install Polygon3)')
	return triangulatePolygon_pypoly2tri(polygon)

def triangulatePolygon_pypoly2tri(polygon):
	polyline = []
	for c in polygon.exterior.coords[:-1]:
		polyline.append(pypoly2tri.shapes.Point(c[0],c[1]))
	cdt = pypoly2tri.cdt.CDT(polyline)
	for i in polygon.interiors:
		polyline = []
		for c in i.coords[:-1]:
			polyline.append(pypoly2tri.shapes.Point(c[0],c[1]))
		cdt.AddHole(polyline)
	try:
		cdt.Triangulate()
	except RecursionError:		# polygon too big for pypoly2tri
		raise TriangulationError('pypoly2tri unable to triangulate large polygon; for '
		                         'non-commercial use, try "pip install Polygon3"')

	triangles = list()
	for t in cdt.GetTriangles():
		triangles.append(shapely.geometry.Polygon([
			t.GetPoint(0).toTuple(),
			t.GetPoint(1).toTuple(),
			t.GetPoint(2).toTuple()
		]))
	return triangles

def triangulatePolygon_gpc(polygon):
	import Polygon as PolygonLib
	poly = PolygonLib.Polygon(polygon.exterior.coords)
	for interior in polygon.interiors:
		poly.addContour(interior.coords, True)
	tristrips = poly.triStrip()
	triangles = []
	for strip in tristrips:
		a, b = strip[:2]
		for c in strip[2:]:
			tri = shapely.geometry.Polygon((a, b, c))
			triangles.append(tri)
			a = b
			b = c
	return triangles

def plotPolygon(polygon, plt, style='r-', **kwargs):
	def plotCoords(chain):
		x, y = chain.xy
		plt.plot(x, y, style, **kwargs)
	if polygon.is_empty:
		return
	if isinstance(polygon, (shapely.geometry.MultiPolygon,
	                        shapely.geometry.MultiLineString,
	                        shapely.geometry.MultiPoint,
	                        shapely.geometry.collection.GeometryCollection)):
		polygons = polygon
	else:
		polygons = [polygon]
	for polygon in polygons:
		if isinstance(polygon, shapely.geometry.Polygon):
			plotCoords(polygon.exterior)
			for ring in polygon.interiors:
				plotCoords(ring)
		elif isinstance(polygon, (shapely.geometry.LineString, shapely.geometry.LinearRing,
		                          shapely.geometry.Point)):
			plotCoords(polygon)
		else:
			raise RuntimeError(f'unknown kind of shapely geometry {polygon}')

class RotatedRectangle:
	"""mixin providing collision detection for rectangular objects and regions"""
	def containsPoint(self, point):
		diff = point - self.position.toVector()
		x, y = diff.rotatedBy(-self.heading)
		return abs(x) <= self.hw and abs(y) <= self.hh

	def intersects(self, rect):
		if not isinstance(rect, RotatedRectangle):
			raise RuntimeError(f'tried to intersect RotatedRectangle with {type(rect)}')
		# Quick check by bounding circles
		dx, dy = rect.position.toVector() - self.position.toVector()
		rr = self.radius + rect.radius
		if (dx * dx) + (dy * dy) > (rr * rr):
			return False
		# Check for separating line parallel to our edges
		if self.edgeSeparates(self, rect):
			return False
		# Check for separating line parallel to rect's edges
		if self.edgeSeparates(rect, self):
			return False
		return True

	@staticmethod
	def edgeSeparates(rectA, rectB):
		"""Whether an edge of rectA separates it from rectB"""
		base = rectA.position.toVector()
		rot = -rectA.heading
		rc = [(corner - base).rotatedBy(rot) for corner in rectB.corners]
		x, y = zip(*rc)
		minx, maxx = findMinMax(x)
		miny, maxy = findMinMax(y)
		if maxx < -rectA.hw or rectA.hw < minx:
			return True
		if maxy < -rectA.hh or rectA.hh < miny:
			return True
		return False

	@property
	def polygon(self):
		if any(needsSampling(c) or needsLazyEvaluation(c) for c in self.corners):
			return None		# can only convert fixed Regions to Polygons
		return self.getConstantPolygon()

	@utils.cached
	def getConstantPolygon(self):
		assert not any(needsSampling(c) or needsLazyEvaluation(c) for c in self.corners)
		# TODO refactor???
		corners = [(x, y) for x, y in self.corners]		# convert Vectors to tuples
		return shapely.geometry.Polygon(corners)
