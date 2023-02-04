"""Objects representing regions in space."""

import math
import random
import itertools

import numpy
import shapely.geometry
import shapely.ops
import shapely.prepared

from scenic.core.distributions import (Samplable, RejectionException, needsSampling,
                                       distributionMethod, smt_add, smt_subtract, smt_multiply, 
                                       smt_divide, smt_and, smt_equal, smt_assert, findVariableName,
                                       checkAndEncodeSMT, writeSMTtoFile, cacheVarName, smt_lessThan, smt_lessThanEq,
                                       smt_ite, normalizeAngle_SMT, smt_or, vector_operation_smt, Options, isConditioned,
                                       Options, UniformDistribution, Constant)
from scenic.core.lazy_eval import valueInContext
from scenic.core.vectors import Vector, OrientedVector, VectorDistribution, VectorField, VectorOperatorDistribution
from scenic.core.geometry import _RotatedRectangle
from scenic.core.geometry import sin, cos, hypot, findMinMax, pointIsInCone, averageVectors
from scenic.core.geometry import headingOfSegment, triangulatePolygon, plotPolygon, polygonUnion
from scenic.core.type_support import toVector
from scenic.core.utils import cached, cached_property, areEquivalent
import matplotlib.pyplot as plt
from scenic.core.type_support import TypecheckedDistribution
from scenic.core.errors import RuntimeParseError
from scenic.core.utils import areEquivalent


def VectorToTuple(vector):
	return (vector.x, vector.y)

def pruneValidLines(smt_file_path, cached_variables, lineString, intersectingPolygon, debug=False):
	""" 
	Output = outputs a list of triangle polygons in `region_polygon` that intersects with ego's visible region (=sector region)

	Input : 
	ego_position := Vector as defined in Scenic
	lineString := Shapely lineString or MultilineString
	view_angle := view cone angle in degrees
	radius := meters
	resolution := in degrees, with how many points to approximate a circle

	"""
	if debug:
		print( "pruneValidLines")

	# if not 'ego_visibleRegion' in cached_variables.keys():
	# 	if debug:
	# 		writeSMTtoFile(smt_file_path, "ego_visibleRegion not in cached_variables.keys()")
	# 	center = cached_variables['ego']
	# 	radius = cached_variables['ego_view_radius']
	# 	heading = cached_variables['ego'].heading
	# 	angle = cached_variables['ego_viewAngle'] * math.pi/180
	# 	sectorRegion = SectorRegion(center, radius, heading, angle)
	# 	cached_variables['ego_visibleRegion'] = sectorRegion
	# 	sector = sectorRegion.polygon
	# 	cached_variables['ego_sector_polygon'] = sector
	# else:
	# 	if debug:
	# 		writeSMTtoFile(smt_file_path, "ego_visibleRegion already in cached_variables.keys()")
	# 	sector = cached_variables['ego_visibleRegion'].polygon

	lineString_list = []
	if isinstance(lineString, shapely.geometry.LineString):
		if not lineString.intersection(intersectingPolygon).is_empty:
			lineString_list.append(lineString)
	elif isinstance(lineString, shapely.geometry.MultiLineString):
		for line in list(lineString.geoms):
			intersection = line.intersection(intersectingPolygon)
			if not intersection.is_empty:
				lineString_list.append(line)

	return lineString_list

def encodeLine_SMT(smt_file_path, cached_variables, lineString_list, point, debug=False):
	""" point1, point2 of type := (float, float), defines a line segment
	""" 
	if debug:
		print( "encodeLine_SMT")

	if lineString_list == []:
		if debug:
			print("encodeLine_SMT() lineString_list is empty")
		return None

	# # parse by line segments:
	line_segments = []
	for line in lineString_list:
		line_segments.append(list(line.coords))

	if debug:
		for line in line_segments:
			x_coords = []
			y_coords = []
			for pt in line:
				x_coords.append(pt[0])
				y_coords.append(pt[1])
			plt.plot(x_coords, y_coords, color ='r')
		plt.plot(*cached_variables['regionAroundEgo_polygon'].exterior.xy, color='g')

	count = 0
	for lineSeg in line_segments:
		if len(lineSeg) < 2:
			continue
		for i in range(len(lineSeg)-1):
			x1, x2 = lineSeg[i][0], lineSeg[i+1][0]
			y1, y2 = lineSeg[i][1], lineSeg[i+1][1]
			x = [x1, x2]
			y = [y1, y2]
			x.sort()
			y.sort()
			smt_x = [str(i) for i in x]
			smt_y = [str(i) for i in y]

			x_range_smt = smt_and(smt_lessThan(smt_x[0], point[0]), smt_lessThanEq(point[0], smt_x[1]))
			y_range_smt = smt_and(smt_lessThan(smt_y[0], point[1]), smt_lessThanEq(point[1], smt_y[1]))
			range_condition = smt_and(x_range_smt, y_range_smt)
			slope = float(y2-y1) / float(x2-x1)
			offset = y1 - slope * x1 # b = y - a*x
			line_encoding = smt_add(smt_multiply(str(slope), point[0]), str(offset)) # ax + b

			if count == 0:
				smt_encoding = smt_ite(range_condition, line_encoding, '-1000')
			else:
				smt_encoding = smt_ite(range_condition, line_encoding, smt_encoding)

	writeSMTtoFile(smt_file_path, smt_assert("equal", point[1], smt_encoding))
	writeSMTtoFile(smt_file_path, smt_assert("not", smt_equal(point[1], '-1000')))
	return point

def encodePolygonalRegion_SMT(smt_file_path, cached_variables, triangles, smt_var, debug=False):
	""" Assumption: the polygons given from polygon region will always be in triangles """
	if debug:
		print( "encodePolygonalRegion_SMT")

	assert(isinstance(smt_var, tuple) and len(smt_var)==2)
	(x, y) = smt_var

	regionAroundEgo = cached_variables['regionAroundEgo']
	regionAroundEgo.encodeToSMT(smt_file_path, cached_variables, (x,y), debug=debug)

	s = findVariableName(smt_file_path, cached_variables, 's', debug=debug)
	t = findVariableName(smt_file_path, cached_variables, 't', debug=debug)

	triangle_list = triangles if isinstance(triangles, list) else list(triangles)
	cumulative_smt_encoding = None

	# 0 <= s <= 1
	s_constraint = smt_assert("and", smt_lessThanEq("0",s), smt_lessThanEq(s,"1"))
	t_constraint = smt_assert("and", smt_lessThanEq("0",t), smt_lessThanEq(t,"1"))
	s_and_t_constraint = smt_assert("<=", smt_add(s,t), "1")
	writeSMTtoFile(smt_file_path, s_constraint)
	writeSMTtoFile(smt_file_path, t_constraint)
	writeSMTtoFile(smt_file_path, s_and_t_constraint)

	for triangle in triangle_list:

		""" barycentric coordinate approach
		A point p is located within a triangle region (defined by p0, p1, p2) if there exists s and t, 
		where 0 <= s <= 1 and 0 <= t <= 1 and s + t <= 1, 
		p = p0 + (p1 - p0) * s + (p2 - p0) * t
		s, t, 1 - s - t are called the barycentric coordinates of the point p 
		"""

		coords = list(triangle.exterior.coords)

		p0 = coords[0]
		p1 = coords[1]
		p2 = coords[2]

		p1_p0 = vector_operation_smt(p1, "subtract", p0)
		multiply1 = vector_operation_smt(p1_p0, "multiply", s)
		p2_p0 = vector_operation_smt(p2, "subtract", p0)
		multiply2 = vector_operation_smt(p2_p0, "multiply", t)
		summation = vector_operation_smt(multiply1, "add", multiply2)
		p = vector_operation_smt(p0, "add" ,summation)
		
		equality_x = smt_equal(x, p[0])
		equality_y = smt_equal(y, p[1])
		smt_encoding = smt_and(equality_x, equality_y)

		if cumulative_smt_encoding == None:
			cumulative_smt_encoding = smt_encoding
		else:
			cumulative_smt_encoding = smt_or(cumulative_smt_encoding, smt_encoding)

		if debug:
			plt.plot(*triangle.exterior.xy, color='g')

		# if debug:
		# 	print( "p0: "+str(p0))
		# 	print( "p1: "+str(p1))
		# 	print( "p2: "+str(p2))

	if debug:
		ego_polygon = cached_variables['regionAroundEgo_polygon']
		plt.plot(*ego_polygon.exterior.xy, color = 'r')
		# center = regionAroundEgo.center
		# plt.plot(center.x, center.y, color='ro')
		plt.show()

	final_smt_encoding = smt_assert(None, cumulative_smt_encoding)
	writeSMTtoFile(smt_file_path, final_smt_encoding)
	return (x,y)

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
	# assert obj.is_valid, obj
	assert obj
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

	def conditionforSMT(self, condition, conditioned_bool):
		if isinstance(self.region, Samplable) and not isConditioned(self.region):
			self.region.conditionforSMT(condition, conditioned_bool)
		return None

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug=False, encode=True):
		if debug:
			print( "PointInRegionDistribution")

		if encode and self in cached_variables.keys():
			if debug:
				print( "PointInRegionDistribution already cached")
			return cached_variables[self]

		if encode and isinstance(self._conditioned, Vector):
			if debug:
				print( "PointInRegionDistribution is conditioned : " + str(self._conditioned))
			vector = self._conditioned
			smt_var = (str(vector.x), str(vector.y))
			return cacheVarName(cached_variables, self, smt_var)

		if smt_var is None:
			if not encode and self in cached_variables.keys():
				x, y = cached_variables[self]
			else:
				x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
				y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)

		if isinstance(self.region, TypecheckedDistribution):
			region = self.region.dist
		else:
			region = self.region

		if isinstance(region, UniformDistribution):
			if debug:
				print("PointInRegionDistribution UniformDistribution Case")
			possibleRegions = region.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=False)
			import scenic.core.distributions as dist
			import scenic.domains.driving.roads as roads
			assert(isinstance(possibleRegions, dist.Options) and possibleRegions.checkOptionsType(roads.NetworkElement))

			if possibleRegions is None:
				return None

			for reg in possibleRegions.options: # UniformDist encodeToSMT outputs Options class
				reg.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug)
				# smt_encoding = vector_operation_smt(smt_var, "equal", point)
				# smt_encoding = smt_assert(None, smt_encoding)
				# writeSMTtoFile(smt_file_path, smt_encoding)

		elif isinstance(region, Options):
			if debug:
				print("PointInRegionDistribution Options Case")
			import scenic.domains.driving.roads as roads
			if encode and region._conditioned.checkOptionsType(roads.NetworkElement):
				region.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug)
			elif not encode and region._conditioned.checkOptionsType(roads.NetworkElement):
				optionRegions = region.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug, encode = encode)
				return optionRegions
			else:
				raise NotImplementedError

		elif isinstance(region, Region):
			if debug:
				print("PointInRegionDistribution Region Case region: ",type(region))
			output = region.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug, encode=encode)
			if not encode:
				return output
		
		else:
			if debug:
				print("PointInRegionDistribution Else Case")
				print("type(region): ", type(region))
			point = region.encodeToSMT(smt_file_path, cached_variables, debug=debug)
			if point is None:
				return None
			(x_cond, y_cond) = vector_operation_smt(point, "equal", smt_var)
			writeSMTtoFile(smt_file_path, smt_assert(None, smt_and(x_cond, y_cond)))

		return cacheVarName(cached_variables, self, smt_var)

	def sampleGiven(self, value):
		return value[self.region].uniformPointInner()

	def isEquivalentTo(self, other):
		if not type(other) is PointInRegionDistribution:
			return False
		return (areEquivalent(self.region, other.region))

	@property
	def heading(self):
		if self.region.orientation is not None:
			return self.region.orientation[self]
		else:
			return 0

	def __repr__(self):
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
			orientation = self.orientation
			if orientation is None:
				orientation = other.orientation
			elif other.orientation is not None:
				orientation = None 		# would be ambiguous, so pick no orientation
			return IntersectionRegion(self, other, orientation=orientation)
		else:
			return other.intersect(self, triedReversed=True)

	def intersects(self, other):
		"""Check if this `Region` intersects another."""
		raise NotImplementedError

	def difference(self, other):
		"""Get a `Region` representing the difference of this one and another."""
		if isinstance(other, EmptyRegion):
			return self
		return DifferenceRegion(self, other)

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

	def distanceTo(self, point):
		raise NotImplementedError

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
		s = f'<{type(self).__name__}'
		if self.name:
			s += f' {self.name}'
		return s + '>'

	def __repr__(self):
		s = f'<{type(self).__name__}'
		if self.name:
			s += f' {self.name}'
		return s + f' at {hex(id(self))}>'

class AllRegion(Region):
	"""Region consisting of all space."""
	def intersect(self, other, triedReversed=False):
		return other

	def containsPoint(self, point):
		return True

	def containsObject(self, obj):
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

	def union(self, other, triedReversed=False):
		return other

	def uniformPointInner(self):
		raise RejectionException(f'sampling empty Region')

	def containsPoint(self, point):
		return False

	def containsObject(self, obj):
		return False

	def distanceTo(self, point):
		return float('inf')

	def show(self, plt, style=None, **kwargs):
		pass

	def __eq__(self, other):
		return type(other) is EmptyRegion

	def __hash__(self):
		return hash(EmptyRegion)

everywhere = AllRegion('everywhere')
nowhere = EmptyRegion('nowhere')

class CircularRegion(Region):
	def __init__(self, center, radius, resolution=32, name=None):
		super().__init__(name, center, radius)
		self.center = center.toVector()
		self.radius = radius
		self.circumcircle = (self.center, self.radius)
		self.resolution = resolution

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug=False):
		if self in cached_variables.keys():
			if debug:
				print( "SectorRegion already cached")
			pt_smt = cached_variables[self]
			if smt_var is None:
				return pt_smt
			else:
				smt = smt_assert(None, vector_operation_smt(smt_var, "equal", pt_smt))
				writeSMTtoFile(smt_file_path, smt)
				return smt_var
		# Instantiate variables to return
		if smt_var is None:
			x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
			y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)

		(output_x, output_y) = smt_var

		# Check whether there are any other distributions to encode first
		center = checkAndEncodeSMT(smt_file_path, cached_variables, self.center, debug=debug)
		(center_x, center_y) = (center[0], center[1])
		radius = checkAndEncodeSMT(smt_file_path, cached_variables, self.radius, debug=debug)

		# Encode and write to file, a contraint for a circle
		if debug: 
			print( "encode circle of ego_visibleRegion")
		shifted_output_x = smt_subtract(output_x, center_x)
		shifted_output_y = smt_subtract(output_y, center_y)
		square_center_x = smt_multiply(shifted_output_x, shifted_output_x)
		square_center_y = smt_multiply(shifted_output_y, shifted_output_y)
		square_radius = smt_multiply(radius, radius)
		summation  = smt_add(square_center_x, square_center_y)
		circle_smt = smt_lessThanEq(summation, square_radius)
		writeSMTtoFile(smt_file_path, smt_assert(None, circle_smt))

		return cacheVarName(cached_variables, self, smt_var)

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	@cached_property
	def polygon(self):
		assert not (needsSampling(self.center) or needsSampling(self.radius))
		ctr = shapely.geometry.Point(self.center)
		return ctr.buffer(self.radius, resolution=self.resolution)

	def sampleGiven(self, value):
		return CircularRegion(value[self.center], value[self.radius],
		                      name=self.name, resolution=self.resolution)

	def evaluateInner(self, context):
		center = valueInContext(self.center, context)
		radius = valueInContext(self.radius, context)
		return CircularRegion(center, radius,
		                      name=self.name, resolution=self.resolution)

	def containsPoint(self, point):
		point = point.toVector()
		return point.distanceTo(self.center) <= self.radius

	def distanceTo(self, point):
		return max(0, point.distanceTo(self.center) - self.radius)

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

	def __repr__(self):
		return f'CircularRegion({self.center}, {self.radius})'

class SectorRegion(Region):
	def __init__(self, center, radius, heading, angle, resolution=18, name=None):
		self.center = center.toVector()
		self.radius = radius
		self.heading = heading
		self.angle = angle
		super().__init__(name, self.center, radius, heading, angle)
		r = (radius / 2) * cos(angle / 2)
		self.circumcircle = (self.center.offsetRadially(r, heading), r)
		self.resolution = resolution

	def isEquivalentTo(self, other):
		if not type(other) is SectorRegion:
			return False
		return (areEquivalent(self.center, other.center)
			and areEquivalent(self.radius, other.radius)
			and areEquivalent(self.heading, other.heading)
			and areEquivalent(self.angle, other.angle)
			and areEquivalent(self.circumcircle, other.circumcircle)
			and areEquivalent(self.resolution, other.resolution))

	def conditionforSMT(self, condition, conditioned_bool):
		if isinstance(self.center, Samplable) and not isConditioned(self.center):
			self.center.conditionforSMT(condition, conditioned_bool)
		if isinstance(self.radius, Samplable) and not isConditioned(self.radius):
			self.radius.conditionforSMT(condition, conditioned_bool)
		if isinstance(self.heading, Samplable) and not isConditioned(self.heading):
			self.heading.conditionforSMT(condition, conditioned_bool)
		if isinstance(self.angle, Samplable) and not isConditioned(self.angle):
			self.angle.conditionforSMT(condition, conditioned_bool)

		return None

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug=False, encode=True):
		assert(encode is True)
		if debug:
			print( "SectorRegion")
			print( "center: "+str(self.center))
			print( "type(center): "+str(type(self.center)))
			print( "radius: "+str(type(self.radius)))
			print( "heading: "+str(self.heading))
			print( "type(heading): "+str(type(self.heading)))
			print( "angle: "+str(type(self.angle)))

		if self in cached_variables.keys():
			if debug:
				print( "SectorRegion already cached")
			pt_smt = cached_variables[self]
			if smt_var is None:
				return pt_smt
			else:
				x_cond, y_cond = vector_operation_smt(smt_var, "equal", pt_smt)
				smt = smt_assert(None, smt_and(x_cond, y_cond))
				writeSMTtoFile(smt_file_path, smt)
				return smt_var

		""" Let a line defined by two points `a` and `b`, with a -> b vector direction of interest,
		and let c be a point of interest of which we want to find its on either left or right of the line
		D = b - a and T = c - a

		A point c is on the left side of the line if sign(Dx * Ty - Dy * Tx) > 0,
		right side if sign(Dx * Ty - Dy * Tx) < 0
		"""

		# Instantiate variables to return
		if smt_var is None:
			x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
			y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)

		(output_x, output_y) = smt_var

		# Check whether there are any other distributions to encode first
		center = checkAndEncodeSMT(smt_file_path, cached_variables, self.center, debug=debug)
		(center_x, center_y) = (center[0], center[1])
		radius = checkAndEncodeSMT(smt_file_path, cached_variables, self.radius, debug=debug)
		heading = checkAndEncodeSMT(smt_file_path, cached_variables, self.heading, debug=debug)
		angle = checkAndEncodeSMT(smt_file_path, cached_variables, self.angle, debug=debug)

		# Encode and write to file, a contraint for a circle
		if debug: 
			print("SectorRegion center: ", center)
			print("SectorRegion radius: ", radius)
			print("SectorRegion heading: ", heading)
			print("SectorRegion angle: ", angle)
			print( "SectorRegion encode circle of ego_visibleRegion")
		shifted_output_x = smt_subtract(output_x, center_x)
		shifted_output_y = smt_subtract(output_y, center_y)
		square_center_x = smt_multiply(shifted_output_x, shifted_output_x)
		square_center_y = smt_multiply(shifted_output_y, shifted_output_y)
		square_radius = smt_multiply(radius, radius)
		summation  = smt_add(square_center_x, square_center_y)
		circle_smt = smt_lessThanEq(summation, square_radius)
		writeSMTtoFile(smt_file_path, smt_assert(None, circle_smt))

		# get the left position of the sector with respect to the center position
		if debug:
			print("SectorRegion() get the left position of the sector with respect to the center position")

		obj1 = VectorOperatorDistribution('offsetRadially', self.center, \
			[self.radius, self.heading - (self.angle) / 2])
		obj1.encodeToSMT(smt_file_path, cached_variables, debug=debug)
		(right_x, right_y) = cached_variables[obj1]

		# get the right position of the sector with respect to the center position
		if debug:
			print("SectorRegion() get the right position of the sector with respect to the center position")
		obj2 = VectorOperatorDistribution('offsetRadially', self.center, \
			[self.radius, self.heading + (self.angle) / 2])
		obj2.encodeToSMT(smt_file_path, cached_variables, debug=debug)
		(left_x, left_y) = cached_variables[obj2]

		# compute "sign(Dx * Ty - Dy * Tx) <= 0" for a given point to be on the right side of the sector's left line 
		# left_angle = findVariableName(cached_variables, smt_file_path, cached_variables['variables'], 'left_angle')
		if debug:
			print( "SectorRegion encode left_line constraint")

		D_left = vector_operation_smt((left_x, left_y), "subtract", (center_x, center_y))
		T_left = vector_operation_smt((output_x, output_y), "subtract", (center_x, center_y))
		Dx_Ty_left = smt_multiply(D_left[0], T_left[1])
		Dy_Tx_left = smt_multiply(D_left[1], T_left[0])
		subtraction_left = smt_subtract(Dx_Ty_left, Dy_Tx_left)
		right_of_leftLine_constraint = smt_lessThanEq(subtraction_left, "0")
		writeSMTtoFile(smt_file_path, smt_assert(None, right_of_leftLine_constraint))

		# compute "0 <= sign(Dx * Ty - Dy * Tx)" for a given point to be on the left side of the sector's right line
		# right_angle = findVariableName(cached_variables, smt_file_path, cached_variables['variables'], 'right_angle')
		if debug:
			print( "SectorRegion encode right_line constraint")

		D_right = vector_operation_smt((right_x, right_y), "subtract", (center_x, center_y))
		T_right = vector_operation_smt((output_x, output_y), "subtract", (center_x, center_y))
		Dx_Ty_right = smt_multiply(D_right[0], T_right[1])
		Dy_Tx_right = smt_multiply(D_right[1], T_right[0])
		subtraction_right = smt_subtract(Dx_Ty_right, Dy_Tx_right)
		left_of_rightLine_constraint = smt_lessThanEq("0", subtraction_right)
		writeSMTtoFile(smt_file_path, smt_assert(None, left_of_rightLine_constraint))

		return cacheVarName(cached_variables, self, smt_var)

	def checkRandomVar(self):
		if not isinstance(self.center._conditioned, Vector):
			return False
		if not isinstance(self.radius._conditioned, (int, float, Constant)):
			return False
		if not isinstance(self.angle._conditioned, (int, float, Constant)):
			return False
		if not isinstance(self.heading._conditioned, (int, float, Constant)):
			return False
		return True

	@cached_property
	def polygon(self):
		center, radius, heading, angle = self.center, self.radius, self.heading, self.angle
		if not isinstance(self.center, Vector):
			center = self.center.sample()
		if not (isinstance(self.radius, (int, float, Constant))):
			radius = self.radius.sample()
		if not (isinstance(self.heading, (int, float, Constant))):
			heading = self.heading.sample()
		if not (isinstance(self.angle, (int, float, Constant))):
			angle = self.angle.sample()

		# center, radius = self.center, self.radius
		ctr = shapely.geometry.Point(center)
		circle = ctr.buffer(radius, resolution=self.resolution)
		if angle >= math.tau - 0.001:
			return circle
		else:
			# heading = self.heading
			# half_angle = self.angle / 2
			half_angle = angle / 2
			mask = shapely.geometry.Polygon([
			    center,
			    center.offsetRadially(radius, heading + half_angle),
			    center.offsetRadially(2*radius, heading),
			    center.offsetRadially(radius, heading - half_angle)
			])
			return circle & mask

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

	def __repr__(self):
		return f'SectorRegion({self.center},{self.radius},{self.heading},{self.angle})'

class RectangularRegion(_RotatedRectangle, Region):
	def __init__(self, position, heading, width, length, name=None):
		super().__init__(name, position, heading, width, length)
		self.position = position.toVector()
		self.heading = heading
		self.width = width
		self.length = length
		self.hw = hw = width / 2
		self.hl = hl = length / 2
		self.radius = hypot(hw, hl)		# circumcircle; for collision detection
		self.corners = tuple(position.offsetRotated(heading, Vector(*offset))
			for offset in ((hw, hl), (-hw, hl), (-hw, -hl), (hw, -hl)))
		self.circumcircle = (self.position, self.radius)

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):

		raise NotImplementedError

	def sampleGiven(self, value):
		return RectangularRegion(value[self.position], value[self.heading],
			value[self.width], value[self.length],
			name=self.name)

	def evaluateInner(self, context):
		position = valueInContext(self.position, context)
		heading = valueInContext(self.heading, context)
		width = valueInContext(self.width, context)
		length = valueInContext(self.length, context)
		return RectangularRegion(position, heading, width, length,
		                         name=self.name)

	def uniformPointInner(self):
		hw, hl = self.hw, self.hl
		rx = random.uniform(-hw, hw)
		ry = random.uniform(-hl, hl)
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
		        and areEquivalent(other.length, self.length))

	def __repr__(self):
		return f'RectangularRegion({self.position},{self.heading},{self.width},{self.length})'

class PolylineRegion(Region):
	"""Region given by one or more polylines (chain of line segments)"""
	def __init__(self, points=None, polyline=None, orientation=True, name=None):
		if orientation is True:
			orientation = VectorField('Polyline', self.defaultOrientation)
			self.usingDefaultOrientation = True
		else:
			self.usingDefaultOrientation = False
		super().__init__(name, orientation=orientation)
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

	def isEquivalentTo(self, other):
		if not type(other) is PolylineRegion:
			return False
		return (areEquivalent(self.points, other.points)
			and areEquivalent(self.lineString, other.lineString))

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug = False, encode=True):
		if debug:
			print( "PolyLineRegion")

		# if self in cached_variables.keys():
		# 	if debug:
		# 		print( "PolyLineRegion already cached")
		# 	return cached_variables[self]

		if smt_var is None:
			x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
			y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)

		intersectingRegion = cached_variables['regionAroundEgo_polygon']
		lineStringList = pruneValidLines(smt_file_path, cached_variables, self.lineString, intersectingRegion, debug=debug)
		if not encode:
			import scenic.core.distributions as dist
			return dist.Options(lineStringList)
		encodeLine_SMT(smt_file_path, cached_variables, lineStringList, smt_var, debug=debug)
		# return cacheVarName(cached_variables, self, smt_var)
		return smt_var

	@classmethod
	def segmentsOf(cls, lineString):
		if isinstance(lineString, shapely.geometry.LineString):
			segments = []
			points = list(lineString.coords)
			if len(points) < 2:
				raise RuntimeError('LineString has fewer than 2 points')
			last = Vector(*points[0][:2])
			for point in points[1:]:
				point = point[:2]
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

	def defaultOrientation(self, point):
		start, end = self.nearestSegmentTo(point)
		return start.angleTo(end)

	def uniformPointInner(self):
		pointA, pointB = random.choices(self.segments,
		                                cum_weights=self.cumulativeLengths)[0]
		interpolation = random.random()
		x, y = averageVectors(pointA, pointB, weight=interpolation)
		if self.usingDefaultOrientation:
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

	def intersects(self, other):
		poly = toPolygon(other)
		if poly is not None:
			intersection = self.lineString & poly
			return not intersection.is_empty
		return super().intersects(other)

	def difference(self, other):
		poly = toPolygon(other)
		if poly is not None:
			diff = self.lineString - poly
			if (diff.is_empty or
			    not isinstance(diff, (shapely.geometry.LineString,
			                          shapely.geometry.MultiLineString))):
				# TODO handle points!
				return nowhere
			return PolylineRegion(polyline=diff)
		return super().difference(other)

	@staticmethod
	def unionAll(regions):
		regions = tuple(regions)
		if not regions:
			return nowhere
		if any(not isinstance(region, PolylineRegion) for region in regions):
			raise RuntimeError(f'cannot take Polyline union of regions {regions}')
		# take union by collecting LineStrings, to preserve the order of points
		strings = []
		for region in regions:
			string = region.lineString
			if isinstance(string, shapely.geometry.MultiLineString):
				strings.extend(string)
			else:
				strings.append(string)
		newString = shapely.geometry.MultiLineString(strings)
		return PolylineRegion(polyline=newString)

	def containsPoint(self, point):
		return self.lineString.intersects(shapely.geometry.Point(point))

	def containsObject(self, obj):
		return False

	@distributionMethod
	def distanceTo(self, point):
		return self.lineString.distance(shapely.geometry.Point(point))

	@distributionMethod
	def signedDistanceTo(self, point):
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
		return shapely.ops.nearest_points(self.lineString, shapely.geometry.Point(point))[0]

	@distributionMethod
	def nearestSegmentTo(self, point):
		dist = self.lineString.project(shapely.geometry.Point(point))
		# TODO optimize?
		for segment, cumLen in zip(self.segments, self.cumulativeLengths):
			if dist <= cumLen:
				break
		# FYI, could also get here if loop runs to completion due to rounding error
		return (Vector(*segment[0]), Vector(*segment[1]))

	def pointAlongBy(self, distance, normalized=False):
		pt = self.lineString.interpolate(distance, normalized=normalized)
		return Vector(pt.x, pt.y)

	def equallySpacedPoints(self, spacing, normalized=False):
		if normalized:
			spacing *= self.length
		return [self.pointAlongBy(d) for d in numpy.arange(0, self.length, spacing)]

	@property
	def length(self):
		return self.lineString.length

	def getAABB(self):
		xmin, ymin, xmax, ymax = self.lineString.bounds
		return ((xmin, ymin), (xmax, ymax))

	def show(self, plt, style='r-', **kwargs):
		plotPolygon(self.lineString, plt, style=style, **kwargs)

	def __getitem__(self, i):
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
				strings.extend(string)
			else:
				strings.append(string)
		newString = shapely.geometry.MultiLineString(strings)
		return PolylineRegion(polyline=newString)

	def __len__(self):
		return len(self.points)

	def __repr__(self):
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
	def __init__(self, points=None, polygon=None, orientation=None, name=None):
		super().__init__(name, orientation=orientation)
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
		# if not self.polygons.is_valid:
		# 	raise RuntimeError('tried to create PolygonalRegion with '
		# 	                   f'invalid polygon {self.polygons}')

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

	def isEquivalentTo(self, other):
		if not type(other) is PolygonalRegion:
			return False
		return (areEquivalent(self.points, other.points)
			and areEquivalent(self.polygons, other.polygons))


	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug=False):
		if debug:
			print( "Class PolygonalRegion")

		import shapely.geometry.polygon as polygon
		assert(not isinstance(self.polygons, polygon.Polygon))

		if self in cached_variables.keys():
			if debug:
				print( "PolygonalRegion already cached")
			pt_smt = cached_variables[self]
			if debug:
				print("pt_smt: ", pt_smt)
			if smt_var is None:
				return pt_smt
			else:
				x_cond, y_cond = vector_operation_smt(smt_var, "equal", pt_smt)
				smt = smt_assert(None, smt_and(x_cond,y_cond))
				writeSMTtoFile(smt_file_path, smt)
				return smt_var

		if smt_var is None:
			x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
			y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)

		polygon_triangles = [triangle[0] for triangle in self.trianglesAndBounds]
		encodePolygonalRegion_SMT(smt_file_path, cached_variables, polygon_triangles, smt_var, debug=debug)
		return cacheVarName(cached_variables, self, smt_var)

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

	def difference(self, other):
		poly = toPolygon(other)
		if poly is not None:
			diff = self.polygons - poly
			if diff.is_empty:
				return nowhere
			elif isinstance(diff, (shapely.geometry.Polygon,
			                       shapely.geometry.MultiPolygon)):
				return PolygonalRegion(polygon=diff, orientation=self.orientation)
			elif isinstance(diff, shapely.geometry.GeometryCollection):
				polys = []
				for geom in diff:
					if isinstance(geom, shapely.geometry.Polygon):
						polys.append(geom)
				if len(polys) == 0:
					# TODO handle points, lines
					raise RuntimeError('unhandled type of polygon difference')
				diff = shapely.geometry.MultiPolygon(polys)
				return PolygonalRegion(polygon=diff, orientation=self.orientation)
			else:
				# TODO handle points, lines
				raise RuntimeError('unhandled type of polygon difference')
		return super().difference(other)

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

	def intersects(self, other):
		poly = toPolygon(other)
		if poly is not None:
			intersection = self.polygons & poly
			return not intersection.is_empty
		return super().intersects(other)

	def union(self, other, triedReversed=False, buf=0):
		poly = toPolygon(other)
		if not poly:
			return super().union(other, triedReversed)
		union = polygonUnion((self.polygons, poly), buf=buf)
		orientation = VectorField.forUnionOf((self, other))
		return PolygonalRegion(polygon=union, orientation=orientation)

	@staticmethod
	def unionAll(regions, buf=0):
		regs, polys = [], []
		for reg in regions:
			if reg != nowhere:
				regs.append(reg)
				polys.append(toPolygon(reg))
		if not polys:
			return nowhere
		if any(not poly for poly in polys):
			raise RuntimeError(f'cannot take union of regions {regions}')
		union = polygonUnion(polys, buf=buf)
		orientation = VectorField.forUnionOf(regs)
		return PolygonalRegion(polygon=union, orientation=orientation)

	@property
	def boundary(self):
		return PolylineRegion(polyline=self.polygons.boundary)

	@cached_property
	def prepared(self):
		return shapely.prepared.prep(self.polygons)

	def containsPoint(self, point):
		return self.prepared.intersects(shapely.geometry.Point(point))

	def containsObject(self, obj):
		objPoly = obj.polygon
		if objPoly is None:
			raise RuntimeError('tried to test containment of symbolic Object!')
		# TODO improve boundary handling?
		return self.prepared.contains(objPoly)

	def containsRegion(self, other, tolerance=0):
		poly = toPolygon(other)
		if poly is None:
			raise RuntimeError('cannot test inclusion of {other} in PolygonalRegion')
		return self.polygons.buffer(tolerance).contains(poly)

	@distributionMethod
	def distanceTo(self, point):
		return self.polygons.distance(shapely.geometry.Point(point))

	def getAABB(self):
		xmin, xmax, ymin, ymax = self.polygons.bounds
		return ((xmin, ymin), (xmax, ymax))

	def show(self, plt, style='r-', **kwargs):
		plotPolygon(self.polygons, plt, style=style, **kwargs)

	def __repr__(self):
		return f'PolygonalRegion({self.polygons})'

	def __eq__(self, other):
		if type(other) is not PolygonalRegion:
			return NotImplemented
		return (other.polygons == self.polygons
		        and other.orientation == self.orientation)

	# @cached
	def __hash__(self):
		# TODO better way to hash mutable Shapely geometries? (also for PolylineRegion)
		return hash((str(self.polygons), self.orientation))

	def __getstate__(self):
		state = self.__dict__.copy()
		state.pop('_cached_prepared', None)		# prepared geometries are not picklable
		return state

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
		tolerance (float; optional): distance tolerance for checking whether a point lies
		  in the region
	"""

	def __init__(self, name, points, kdTree=None, orientation=None, tolerance=1e-6):
		super().__init__(name, orientation=orientation)
		self.points = tuple(points)
		for point in self.points:
			if needsSampling(point):
				raise RuntimeError('only fixed PointSetRegions are supported')
		import scipy.spatial	# slow import not often needed
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

	@distributionMethod
	def distanceTo(self, point):
		distance, _ = self.kdTree.query(point)
		return distance

	def __eq__(self, other):
		if type(other) is not PointSetRegion:
			return NotImplemented
		return (other.name == self.name
		        and other.points == self.points
		        and other.orientation == self.orientation)

	@cached
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
	def __init__(self, *regions, orientation=None, sampler=None, name=None):
		self.regions = tuple(regions)
		if len(self.regions) < 2:
			raise RuntimeError('tried to take intersection of fewer than 2 regions')
		super().__init__(name, *self.regions, orientation=orientation)
		if sampler is None:
			sampler = self.genericSampler
		self.sampler = sampler

	def conditionforSMT(self, condition, conditioned_bool):
		for region in self.regions:
			if isinstance(region, Samplable) and not isConditioned(region):
				region.conditionforSMT(condition, conditioned_bool)

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug=False, encode=True):
		if debug:
			print( "IntersectionRegion")
			
		if encode and self in cached_variables.keys():
			if debug:
				print( "IntersectionRegion already exists in cached_variables")
			output_var = cached_variables[self]
			return output_var

		if not encode:
			sectorRegion, polylineRegion, polygonalRegion = None, None, None
			for region in self.regions:
				if isinstance(region, SectorRegion):
					sectorRegion = region
				if isinstance(region, PolylineRegion):
					polylineRegion = region 
				if isinstance(region, PolygonalRegion):
					polygonalRegion = region

			if sectorRegion is not None and polylineRegion is not None:
				lineString = polylineRegion.lineString
				intersectingPolygon = sectorRegion.polygon
				import scenic.core.distributions as dist
				output = pruneValidLines(smt_file_path, cached_variables, lineString, intersectingPolygon, debug=False)
				if output is None or output == []:
					return None
				else:
					return dist.Options(output)
			else:
				raise NotImplementedError

		if smt_var is None:
			x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
			y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)

		for region in self.regions:
			region.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug)
			# smt_encoding = vector_operation_smt( smt_var, "equal", output_var)
			# smt_encoding = smt_assert(None, smt_encoding)
			# writeSMTtoFile(smt_file_path, smt_encoding)

		return cacheVarName(cached_variables, self, smt_var)

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
		                          sampler=self.sampler, name=self.name)

	def evaluateInner(self, context):
		regs = (valueInContext(reg, context) for reg in self.regions)
		orientation = valueInContext(self.orientation, context)
		return IntersectionRegion(*regs, orientation=orientation, sampler=self.sampler,
		                          name=self.name)

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
		if len(other.regions)!=len(self.regions):
			return False

		for i in range(len(self.regions)):
			if not areEquivalent(self.regions[i], other.regions[i]):
				return False
		if self.orientation != self.orientation:
			return False

		return True

		# return (areEquivalent(set(other.regions), set(self.regions))
		#         and other.orientation == self.orientation)

	def __repr__(self):
		return f'IntersectionRegion({self.regions})'

class DifferenceRegion(Region):
	def __init__(self, regionA, regionB, sampler=None, name=None):
		self.regionA, self.regionB = regionA, regionB
		super().__init__(name, regionA, regionB, orientation=regionA.orientation)
		if sampler is None:
			sampler = self.genericSampler
		self.sampler = sampler

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		raise NotImplementedError

	def sampleGiven(self, value):
		regionA, regionB = value[self.regionA], value[self.regionB]
		# Now that regions have been sampled, attempt difference again in the hopes
		# there is a specialized sampler to handle it (unless we already have one)
		if self.sampler is self.genericSampler:
			diff = regionA.difference(regionB)
			if not isinstance(diff, DifferenceRegion):
				diff.orientation = value[self.orientation]
				return diff
		return DifferenceRegion(regionA, regionB, orientation=value[self.orientation],
		                        sampler=self.sampler, name=self.name)

	def evaluateInner(self, context):
		regionA = valueInContext(self.regionA, context)
		regionB = valueInContext(self.regionB, context)
		orientation = valueInContext(self.orientation, context)
		return DifferenceRegion(regionA, regionB, orientation=orientation,
		                        sampler=self.sampler, name=self.name)

	def containsPoint(self, point):
		return regionA.containsPoint(point) and not regionB.containsPoint(point)

	def uniformPointInner(self):
		return self.orient(self.sampler(self))

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
		return f'DifferenceRegion({self.regionA}, {self.regionB})'
