
#### GEOMETRY
#### utility functions for geometric computation

import math
import itertools

import numpy as np
import shapely.geometry
import shapely.ops
import Polygon as PolygonLib	# TODO remove dependency (see triangulatePolygon)

from scenic.core.distributions import (needsSampling, distributionFunction,
                                       monotonicDistributionFunction)
from scenic.core.specifiers import needsLazyEvaluation
import scenic.core.utils as utils

@distributionFunction
def sin(x):
	return math.sin(x)

@distributionFunction
def cos(x):
	return math.cos(x)

@monotonicDistributionFunction
def hypot(x, y):
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
	return math.atan2(by - ay, bx - ax) - (math.pi / 2.0)

def viewAngleToPoint(point, base, heading):
	x, y = base
	ox, oy = point
	a = math.atan2(oy - y, ox - x) - (heading + (math.pi / 2.0))
	if a < -math.pi:
		a += math.tau
	elif a > math.pi:
		a -= math.tau
	assert -math.pi <= a and a <= math.pi
	return a

def apparentHeadingAtPoint(point, heading, base):
	x, y = base
	ox, oy = point
	a = (heading + (math.pi / 2.0)) - math.atan2(oy - y, ox - x)
	if a < -math.pi:
		a += math.tau
	elif a > math.pi:
		a -= math.tau
	assert -math.pi <= a and a <= math.pi
	return a

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

def polygonUnion(polys, tolerance=0.05, holeTolerance=0.002):
	union = shapely.ops.unary_union(list(polys))
	assert union.is_valid, union
	if tolerance > 0:
		if isinstance(union, shapely.geometry.MultiPolygon):
			polys = [cleanPolygon(poly, tolerance, holeTolerance) for poly in union]
			union = shapely.ops.unary_union(polys)
		else:
			union = cleanPolygon(union, tolerance, holeTolerance)
	assert union.is_valid, union
	return union

def cleanPolygon(poly, tolerance, holeTolerance):
	exterior = cleanChain(poly.exterior.coords, tolerance)
	assert len(exterior) >= 3
	ints = []
	for interior in poly.interiors:
		interior = cleanChain(interior.coords, tolerance)
		if len(interior) >= 3:
			hole = shapely.geometry.Polygon(interior)
			if hole.area > holeTolerance:
				ints.append(interior)
	newPoly = shapely.geometry.Polygon(exterior, ints)
	assert newPoly.is_valid, newPoly
	return newPoly

def cleanChain(chain, tolerance, angleTolerance=0.008):
	if len(chain) <= 2:
		return chain
	closed = (tuple(chain[0]) == tuple(chain[-1]))
	tol2 = tolerance * tolerance
	# collapse hooks
	chain = np.array(chain)
	a, b = chain[-1], chain[0]
	newChain = []
	for c in chain[1:]:
		dx, dy = c[0] - a[0], c[1] - a[1]
		if (dx * dx) + (dy * dy) > tol2:
			newChain.append(b)
			a = b
			b = c
		else:
			b = c
	if len(newChain) == 0:
		return newChain
	newChain.append(newChain[0] if closed else c)
	return newChain

def triangulatePolygon(polygon):
	# TODO replace with another library!
	# N.B. can't use shapely.ops.triangulate since it doesn't respect edges
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

def plotPolygon(polygon, plt, style='r-'):
	def plotRing(ring):
		coords = ring.coords
		for i, point in enumerate(coords[:-1]):
			nextPt = coords[i+1]
			plt.plot([point[0], nextPt[0]], [point[1], nextPt[1]], style)
	if isinstance(polygon, shapely.geometry.MultiPolygon):
		polygons = polygon
	else:
		polygons = [polygon]
	for polygon in polygons:
		plotRing(polygon.exterior)
		for ring in polygon.interiors:
			plotRing(ring)

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
	@utils.cached
	def polygon(self):
		if needsSampling(self) or needsLazyEvaluation(self):
			return None		# can only convert fixed Regions to Polygons
		# TODO refactor???
		corners = [(x, y) for x, y in self.corners]		# convert Vectors to tuples
		return shapely.geometry.Polygon(corners)
