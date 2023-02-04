"""Objects representing distributions that can be sampled from."""

import collections
import itertools
import random
import math
import typing
import warnings

import numpy
import wrapt

from scenic.core.lazy_eval import (LazilyEvaluable,
    requiredProperties, needsLazyEvaluation, valueInContext, makeDelayedFunctionCall)
from scenic.core.utils import argsToString, areEquivalent, cached, sqrt2
from scenic.core.errors import RuntimeParseError
import matplotlib.pyplot as plt

def smt_add(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(+ "+var1+" "+var2+")"

def smt_subtract(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(- "+var1+" "+var2+")"

def smt_multiply(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(* "+var1+" "+var2+")"

def smt_divide(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(/ "+var1+" "+var2+")"

def smt_and(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(and "+var1+" "+var2+")"

def smt_or(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(or "+var1+" "+var2+")"

def smt_equal(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(= "+var1+" "+var2+")"

def smt_lessThan(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(< "+var1+" "+var2+")"

def smt_lessThanEq(var1, var2):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str))
	return "(<= "+var1+" "+var2+")"

def smt_ite(predicate, output1, output2):
	assert(isinstance(predicate, str))
	assert(isinstance(output1, str))
	assert(isinstance(output2, str))
	return "(ite "+predicate+" "+output1+" "+output2+")"

def smt_not(predicate):
	return "(not "+predicate+")"

def smt_assert(operation_type, var1, var2=None):
	assert(isinstance(var1, str))
	assert(isinstance(var2, str) or var2==None)

	if operation_type == "add":
		op_encoding = smt_add(var1, var2)
	elif operation_type == "subtract":
		op_encoding = smt_subtract(var1, var2)
	elif operation_type == "multiply":
		op_encoding = smt_multiply(var1, var2)
	elif operation_type == "divide":
		op_encoding = smt_divide(var1, var2)
	elif operation_type == "equal":
		op_encoding = smt_equal(var1, var2)
	elif operation_type == "and":
		op_encoding = smt_and(var1, var2)
	# elif operation_type == "mod":
	# 	op_encoding = smt_mod(var1, var2)
	elif operation_type == "<=":
		op_encoding = smt_lessThanEq(var1, var2)
	elif operation_type == "<":
		op_encoding = smt_lessThan(var1, var2)
	elif operation_type == "not":
		op_encoding = smt_not(var1)
	elif operation_type == None:
		op_encoding = var1
	else:
		raise NotImplementedError

	return "(assert "+ op_encoding +")"

def vector_operation_smt(vector1, operation, vector2):
	""" vector1, vector2 := (x, y) from triangle := shapely.geometry.polygon.Polygon
	x, y are floats"""

	if isinstance(vector1, tuple):
		x1 = str(vector1[0])
		y1 = str(vector1[1])
	else:
		assert(isinstance(vector1, str))
		x1 = y1 = vector1

	if isinstance(vector2, tuple):
		x2 = str(vector2[0])
		y2 = str(vector2[1])
	else: # scalar operation to a vector case
		assert(isinstance(vector2, str))
		x2 = y2 = vector2

	if operation == "add":	
		x = smt_add(x1, x2)
		y = smt_add(y1, y2)
	elif operation == "subtract":
		x = smt_subtract(x1, x2)
		y = smt_subtract(y1, y2)
	elif operation == "multiply":
		x = smt_multiply(x1, x2)
		y = smt_multiply(y1, y2) 
	elif operation == "equal":
		x = smt_equal(x1, x2)
		y = smt_equal(y1, y2)
	else:
		raise NotImplementedError

	return (x, y)

def normalizeAngle_SMT(smt_file_path, cached_variables, original_angle, debug=False):
	theta = findVariableName(smt_file_path, cached_variables, 'theta', debug=debug)
	# theta_encoding = smt_assert("equal", theta, angle)          
	# angleTo_encoding1 = smt_assert("equal", theta, smt_ite(smt_lessThanEq("3.1416",original_angle), \
	# 	smt_subtract(original_angle,"6.2832"), original_angle))
	# angleTo_encoding2 = smt_assert("equal", theta, smt_ite(smt_lessThanEq(original_angle, "-3.1416"), \
	# 	smt_add(original_angle,"6.2832"), original_angle))

	angleTo_encoding1 = smt_ite(smt_lessThanEq("3.1416",original_angle), \
		smt_subtract(original_angle,"6.2832"), original_angle)
	angleTo_encoding2 = smt_ite(smt_lessThanEq(original_angle, "-3.1416"), \
		smt_add(original_angle,"6.2832"), angleTo_encoding1)
	smt_encoding = smt_assert("equal", theta, angleTo_encoding2)

	# writeSMTtoFile(smt_file_path, angleTo_encoding1)
	# writeSMTtoFile(smt_file_path, angleTo_encoding2)
	writeSMTtoFile(smt_file_path, smt_encoding)
	return theta

def findVariableName(smt_file_path, cached_variables, class_name, class_type=None, debug=False):
	""" for smt encoding, to avoid duplicate naming, add a number at then end for differentiation 
		returns the next available name """

	variable_list = cached_variables['variables']
	cached_var = [variable for variable in variable_list if variable.startswith(class_name)]
	var_name = class_name+str(len(cached_var)+1) 
	if debug:
		print("findVariableName() creating a variable: "+var_name)

	if class_type == None :
		declare_var= "(declare-fun "+var_name+" () Real)"
	else:
		declare_var= "(declare-fun "+var_name+" () "+class_type+")"

	writeSMTtoFile(smt_file_path, declare_var)
	cached_variables['variables'].append(var_name)
	return var_name

def checkAndEncodeSMT(smt_file_path, cached_variables, obj, debug=False):
	# checks the type of the obj and encode to smt accordingly
	# this step can be done in each class, but having this function
	# saves the trouble of executing the same step, repeatedly
	# print("checkAndEncodeSMT")
	if obj in cached_variables.keys():
		# print("1st case")
		return cached_variables[obj]
	elif isinstance(obj, int) or isinstance(obj, float):
		return str(obj)
	elif isinstance(obj, str):
		# print("2nd case")
		return obj
	elif isinstance(obj, tuple):
		# print("3rd case")
		elements = []
		for elem in obj:
			elements.append(checkAndEncodeSMT(smt_file_path, cached_variables, elem,debug))
		return tuple(elements)
	elif isinstance(obj._conditioned, Constant):
		# print("4th case")
		return str(obj._conditioned.value)
	elif isinstance(obj._conditioned, Samplable):
		# print("5th case")
		return obj._conditioned.encodeToSMT(smt_file_path, cached_variables, debug=debug)
	elif isinstance(obj._conditioned, int) or isinstance(obj._conditioned, float):
		# print("6th case")
		return str(obj._conditioned)
	elif isinstance(obj._conditioned, str):
		# print("7th case")
		# this covers case in regions.py, PointInRegionDist's encodeToSMT where
		# a Vector is instantiated with string variable names
		return obj._conditioned
	else:
		raise NotImplementedError
	return None

def writeSMTtoFile(smt_file_path, smt_encoding):
	# print("writeSMTtoFile(): ", smt_encoding)
	if isinstance(smt_encoding, str):
		with open(smt_file_path, "a+") as smt_file:
			smt_file.write(smt_encoding+"\n")

	elif isinstance(smt_encoding, tuple) and len(smt_encoding) == 2:
		with open(smt_file_path, "a+") as smt_file:
			smt_file.write(smt_encoding[0]+"\n")
			smt_file.write(smt_encoding[1]+"\n")
	else :
		raise NotImplementedError

	return None

def cacheVarName(cached_variables, obj, var_names):
	""" caches variable names.
	type : var_names := tuple """

	# if obj in cached_variables.keys():
	# 	return cached_variables[obj]
	if not isinstance(var_names, tuple):
		var_names = (var_names)

	if obj in set(cached_variables.keys()):
		return cached_variables[obj]

	for var in var_names:
		if var not in cached_variables['variables']:
			cached_variables['variables'].append(var)

	if len(var_names) == 1:
		var_names = var_names[0]

	cached_variables[obj] = var_names
	return var_names


def refineCenterlinePts(centerlinePts, elem, intersection):
	import scenic.core.vectors as vectors
	import shapely.geometry
	import scenic.domains.driving.roads as roads

	refinedCenterlinePts = []
	if isinstance(elem, roads.NetworkElement):
		if len(centerlinePts) == 0:
			# find the two points closest to the intersection 
			distance = [shapely.geometry.point.Point(pt[0],pt[1]).distance(intersection) for pt in elem.centerline.points]
			centerlinePts = [elem.centerline.points[i] for i in range(len(distance))]
			if len(centerlinePts) == 0:
				centerlinePts = [centerlinePts[centerlinePts.index(min(distance))]]

		if len(centerlinePts) == 1:
			if centerlinePts[0] == elem.centerline.points[0]:
				centerlinePts.append(elem.centerline.points[1])
				return centerlinePts
			else:
				index = elem.centerline.points.index(centerlinePts[0])
				return elem.centerline.points[index-1:index+2]

	elif isinstance(elem, shapely.geometry.LineString):
		if len(centerlinePts) < 2:
			return None
		
	else:
		raise NotImplementedError

	prev_heading = None # (key, value) = (point, slope with the next point), last point's slope is None
	for i in range(len(centerlinePts)-1):
		x1, y1 = centerlinePts[i]
		x2, y2 = centerlinePts[i+1]
		heading = vectors.Vector(x1,y1).angleTo(vectors.Vector(x2,y2))
		if i == len(centerlinePts)-2:
			if (x1,y1) not in refinedCenterlinePts:
				refinedCenterlinePts.append((x1,y1))
				refinedCenterlinePts.append((x2,y2))
				break

		if prev_heading is None:
			prev_heading = heading
			refinedCenterlinePts.append((x1,y1))
		else:
			angle_difference = abs(heading-prev_heading)
			if 0.1745 < angle_difference and angle_difference < math.pi/2: # radians (about 5 degrees)
			# if 0.01745 < angle_difference and angle_difference < math.pi/2: # radians (about 1 degrees)
				refinedCenterlinePts.append((x1,y1))
				prev_heading = heading
			else:
				continue
	assert(len(refinedCenterlinePts)>=2)
	return refinedCenterlinePts

def encodePolyLineHeading(cached_variables, elems, smt_file_path, point, heading_var, debug = False):
	import scenic.core.vectors as vectors

	for elem in elems:
		centerlinePts = list(elem.coords)
		intersection = elem.intersection(cached_variables['regionAroundEgo_polygon'])

		if debug:
			print("len(elem.coords): ", len(elem.coords))
			print("len(centerlinePts): ", len(centerlinePts))

		centerlinePts = refineCenterlinePts(centerlinePts, elem, intersection)
		if centerlinePts is None or centerlinePts == []:
			if debug:
				print("centerlinePts is None")
			continue

		if debug:
			print("refined len(centerlinePts): ",len(centerlinePts))
		
		for i in range(len(centerlinePts)-1):
			start = centerlinePts[i]
			end = centerlinePts[i+1]
			x1, y1 = start[0], start[1]
			x2, y2 = end[0], end[1]

			startVector = vectors.Vector(start[0], start[1])
			endVector = vectors.Vector(end[0], end[1])
			heading = startVector.angleTo(endVector)

			slope = float(y2-y1) / float(x2-x1)
			offset = y1 - slope * x1 # b = y - a*x
			line_encoding = smt_equal(point[1], smt_add(smt_multiply(str(slope), point[0]), str(offset))) # ax + b

			if i == 0:
				smt_encoding = smt_ite(line_encoding, str(heading), '-1000')
			else:
				smt_encoding = smt_ite(line_encoding, str(heading), smt_encoding)

	writeSMTtoFile(smt_file_path, smt_assert("equal", heading_var, smt_encoding))
	writeSMTtoFile(smt_file_path, smt_assert("not", smt_equal(heading_var, '-1000')))
	return heading_var

# find line perpendicular to the centerline segment & pass through the centerline point
def encodeHeading(cached_variables, elems, smt_file_path, smt_var, heading_var, debug=False):
	import shapely.geometry
	import scenic.core.geometry as geometry
	import scenic.core.vectors as vectors
	import scenic.core.regions as regions
	import scenic.domains.driving.roads as roads

	smt_encoding = None
	regionAroundEgo = cached_variables['regionAroundEgo']

	# 0 <= s <= 1
	s = findVariableName(smt_file_path, cached_variables, 's', debug=debug)
	t = findVariableName(smt_file_path, cached_variables, 't', debug=debug)
	s_constraint = smt_assert("and", smt_lessThanEq("0",s), smt_lessThanEq(s,"1"))
	t_constraint = smt_assert("and", smt_lessThanEq("0",t), smt_lessThanEq(t,"1"))
	s_and_t_constraint = smt_assert("<=", smt_add(s,t), "1")
	writeSMTtoFile(smt_file_path, s_constraint)
	writeSMTtoFile(smt_file_path, t_constraint)
	writeSMTtoFile(smt_file_path, s_and_t_constraint)

	if debug:
		print("encodeHeading()")
		print("elems: ", elems)

	for elem in elems:
		centerlinePts, intersection = None, None
		if len(elem.centerline.points) > 2:
			centerlinePts = [pt for pt in elem.centerline.points if regionAroundEgo.containsPoint(vectors.Vector(pt[0],pt[1]))]
		else:
			centerlinePts = elem.centerline.points

		intersection = elem.polygon.intersection(cached_variables['regionAroundEgo_polygon'])
		if debug:
			print("len(elem.centerline.points): ", len(elem.centerline.points))
			print("len(centerlinePts): ", len(centerlinePts))

		# if len(centerlinePts) == 1:
		# 	index = elem.centerline.points.index(centerlinePts[0])
		# 	if index < len(elem.centerline.points)-1:
		# 		centerlinePts.append(elem.centerline.points[index+1])
		# 	else:
		# 		centerlinePts.append(elem.centerline.points[index-1])
		# # assert(len(centerlinePts) >= 2)

		# intersection = elem.polygon.intersection(cached_variables['regionAroundEgo_polygon'])
		centerlinePts = refineCenterlinePts(centerlinePts, elem, intersection)
		if centerlinePts is None or centerlinePts == []:
			if debug:
				print("centerlinePts is None")
			continue

		if debug:
			print("refined len(centerlinePts): ",len(centerlinePts))

		offset_distance = math.sqrt(math.pow(elem.leftEdge.points[0][0]-elem.rightEdge.points[0][0],2)+\
							math.pow(elem.leftEdge.points[0][1]-elem.rightEdge.points[0][1],2))/2 + 5
		
		for i in range(len(centerlinePts)-1):
			# find the previous two points
			if i==0:
				prevCenterPt = centerlinePts[0]
				center_point = shapely.geometry.Point(prevCenterPt[0],prevCenterPt[1])

			x1,y1 = [prevCenterPt[0], prevCenterPt[1]]
			x2,y2 = [centerlinePts[i+1][0], centerlinePts[i+1][1]]

			if x2 == x1:
				slope = 100
			else:
				slope = float(y2-y1)/float(x2-x1)

			if abs(slope) > 20:
			    # find two points on the perpendicular line
				left_x, right_x = x2 - offset_distance, x2 + offset_distance
				left_y, right_y = y2, y2
				if i == 0:
					prevLeftPt = [x1 - offset_distance, y1]
					prevRightPt = [x1 + offset_distance, y1]

			elif abs(slope) < 0.2:
				left_x, right_x = x2, x2
				left_y, right_y = y2 - offset_distance, y2 + offset_distance
				if i == 0:
					prevLeftPt = [x1, y1 - offset_distance]
					prevRightPt = [x1, y1 + offset_distance]

			else:
				perpendicular_slope = -1 / float(slope)
				bias = y2 - perpendicular_slope * x2
				left_x, right_x = x2 - offset_distance, x2 + offset_distance
				left_y = perpendicular_slope * left_x + bias
				right_y = perpendicular_slope * right_x + bias
				if i == 0:
					bias = y1 - perpendicular_slope * x1
					left_x0, right_x0 = x1 - offset_distance, x1 + offset_distance
					left_y0 = perpendicular_slope * left_x + bias
					right_y0 = perpendicular_slope * right_x + bias
					prevLeftPt = [left_x0, left_y0]
					prevRightPt = [right_x0, right_y0]

			left_pt = (left_x, left_y)
			right_pt = (right_x, right_y)
			line = shapely.geometry.LineString([left_pt, right_pt])

			intersect_leftPt = elem.leftEdge.lineString.intersection(line)
			intersect_rightPt = elem.rightEdge.lineString.intersection(line)

			if isinstance(intersect_leftPt, (shapely.geometry.MultiPoint, shapely.geometry.LineString)):
				intersect_leftPt = findClosestPoint(intersect_leftPt, elem.leftEdge, \
										    shapely.geometry.Point(x2, y2), debug = debug)
			if isinstance(intersect_rightPt, (shapely.geometry.MultiPoint, shapely.geometry.LineString)):
				intersect_rightPt = findClosestPoint(intersect_rightPt, elem.rightEdge, \
											shapely.geometry.Point(x2, y2), debug = debug)
			if isinstance(intersect_leftPt, shapely.geometry.point.Point):
				intersect_leftPt = (intersect_leftPt.x, intersect_leftPt.y)
			if isinstance(intersect_rightPt, shapely.geometry.point.Point):
				intersect_rightPt = (intersect_rightPt.x, intersect_rightPt.y)

			if i == 0:
				line0 = shapely.geometry.LineString([prevLeftPt, prevRightPt])
				prevLeftPt = elem.leftEdge.lineString.intersection(line0)
				prevRightPt = elem.rightEdge.lineString.intersection(line0)

				if isinstance(prevLeftPt, (shapely.geometry.MultiPoint, shapely.geometry.LineString)):
					prevLeftPt = findClosestPoint(prevLeftPt, elem.leftEdge, \
											    shapely.geometry.Point(x1, y1), debug = debug)
				if isinstance(prevRightPt, (shapely.geometry.MultiPoint, shapely.geometry.LineString)):
					prevRightPt = findClosestPoint(prevRightPt, elem.rightEdge, \
												shapely.geometry.Point(x1, y1), debug = debug)
				if isinstance(prevLeftPt, shapely.geometry.point.Point):
					prevLeftPt = (prevLeftPt.x, prevLeftPt.y)
				if isinstance(prevRightPt, shapely.geometry.point.Point):
					prevRightPt = (prevRightPt.x, prevRightPt.y)

			if debug:
				if isinstance(elem.polygon, shapely.geometry.multipolygon.MultiPolygon):
					for e in elem.polygon.geoms:
						plt.plot(*e.exterior.xy)
				else:
					plt.plot(*elem.polygon.exterior.xy)
				for pt in centerlinePts:
					[x,y] = pt
					plt.plot(x,y,'ko')

				plt.plot(prevLeftPt[0], prevLeftPt[1], 'rx')
				plt.plot(prevRightPt[0], prevRightPt[1], 'rx')
				plt.plot(x1, y1,'ro')
				plt.plot(left_x, left_y, 'bx')
				plt.plot(right_x, right_y, 'bx')
				plt.plot(x2, y2, 'go')
				plt.plot(intersect_leftPt[0], intersect_leftPt[1], 'kx')
				plt.plot(intersect_rightPt[0], intersect_rightPt[1], 'kx')
				x_range = [centerlinePts[0][0], centerlinePts[-1][0]]
				y_range = [centerlinePts[0][1], centerlinePts[-1][1]]
				x_left, x_right = min(x_range), max(x_range)
				y_bottom, y_top = min(y_range), max(y_range)
				plt.xlim(left=x_left-20, right=x_right+20)
				plt.ylim(bottom=y_bottom-20, top = y_top+20)
				plt.show()

			# find the intersecting left/right points with left & right edge of the elem
			# if i+1 < len(centerlinePts)-1:
			# 	leftPt = (intersect_leftPt[0], intersect_leftPt[1])
			# 	rightPt = (intersect_rightPt[0], intersect_rightPt[1])
			# else:
			# 	leftPt = elem.leftEdge.lineString.coords[-1]
			# 	rightPt = elem.rightEdge.lineString.coords[-1]
			leftPt = (intersect_leftPt[0], intersect_leftPt[1])
			rightPt = (intersect_rightPt[0], intersect_rightPt[1])

			# Create a Polygon
			square_points = [prevLeftPt, leftPt, rightPt, prevRightPt]
			polygon = shapely.geometry.polygon.Polygon(square_points)
			triangles = geometry.triangulatePolygon(polygon)
			if debug:
				plt.plot(*polygon.exterior.xy)
				print("len(triangles): ", len(triangles))

			# # encode a region left of the vector [prevLeftPt, prevRightPt]
			# leftOf_smt = encodeLeftRightOf(prevLeftPt, prevRightPt, smt_var, smt_file_path, side='left')

			# # encode a region right of the vector [leftPt, rightPt]
			# rightOf_smt = encodeLeftRightOf(leftPt, rightPt, smt_var, smt_file_path, side='right')

			# # encode a region right of [prevLeftPt, leftPt]
			# rightOf_smt2 = encodeLeftRightOf(prevLeftPt, leftPt, smt_var, smt_file_path, side='right')

			# # encode a region left of [prevRightPt, rightPt]
			# leftOf_smt2 = encodeLeftRightOf(prevRightPt, rightPt, smt_var, smt_file_path, side='left')

			# encode heading smt with ite
			# joint_smt = smt_and(smt_and(leftOf_smt, rightOf_smt), smt_and(leftOf_smt2, rightOf_smt2))
			joint_smt = encodePolygon(smt_file_path, cached_variables, triangles, smt_var, s,t, debug=debug)
			heading = str(vectors.Vector(x1,y1).angleTo(vectors.Vector(x2,y2)))

			if debug:
				print("heading: ", heading)

			if smt_encoding is None:
				smt_encoding = smt_ite(joint_smt, heading, '-1000')
			else:
				smt_encoding = smt_ite(joint_smt, heading, smt_encoding)

			assert(smt_encoding is not None)

			# cache previous points
			prevLeftPt = leftPt
			prevRightPt = rightPt
			prevCenterPt = centerlinePts[i+1]

	if smt_encoding is not None:
		writeSMTtoFile(smt_file_path, smt_assert("equal", heading_var , smt_encoding))
		writeSMTtoFile(smt_file_path, smt_assert("not", smt_equal(heading_var, '-1000')))
	return heading_var

def encodePolygon(smt_file_path, cached_variables, triangles, smt_var, s, t, debug=False):
	""" Assumption: the polygons given from polygon region will always be in triangles """
	if debug:
		print( "encodePolygon()")

	assert(isinstance(smt_var, tuple) and len(smt_var)==2)
	(x, y) = smt_var

	triangle_list = triangles if isinstance(triangles, list) else list(triangles)
	cumulative_smt_encoding = None

	# # 0 <= s <= 1
	# s = findVariableName(smt_file_path, cached_variables, 's', debug=debug)
	# t = findVariableName(smt_file_path, cached_variables, 't', debug=debug)
	# s_constraint = smt_assert("and", smt_lessThanEq("0",s), smt_lessThanEq(s,"1"))
	# t_constraint = smt_assert("and", smt_lessThanEq("0",t), smt_lessThanEq(t,"1"))
	# s_and_t_constraint = smt_assert("<=", smt_add(s,t), "1")
	# writeSMTtoFile(smt_file_path, s_constraint)
	# writeSMTtoFile(smt_file_path, t_constraint)
	# writeSMTtoFile(smt_file_path, s_and_t_constraint)

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
		# ego_polygon = cached_variables['regionAroundEgo_polygon']
		# plt.plot(*ego_polygon.exterior.xy, color = 'r')
		# center = regionAroundEgo.center
		# plt.plot(center.x, center.y, color='ro')
		plt.show()

	return cumulative_smt_encoding

def encodeLeftRightOf(leftPt, rightPt, smt_var, smt_file_path, side):
    """ D >= 0 : the point, (xp,yp), is on the left-hand side or the line 
        D = (x2-x1) * (yp-y1) - (xp-x1) * (y2-y1)
    """
    (xp, yp) = smt_var
    (x1,y1) = (str(leftPt[0]), str(leftPt[1]))
    (x2,y2) = (str(rightPt[0]), str(rightPt[1]))
    x2_x1 = smt_subtract(x2,x1)
    yp_y1 = smt_subtract(yp,y1)
    mult1 = smt_multiply(x2_x1, yp_y1)
    xp_x1 = smt_subtract(xp,x1)
    y2_y1 = smt_subtract(y2,y1)
    mult2 = smt_multiply(xp_x1, y2_y1)
    D = smt_subtract(mult1, mult2)
    if side == 'left':
        smt_encoding = smt_lessThanEq('0', D)
    else:
        smt_encoding = smt_lessThanEq(D ,'0')
        
#     writeSMTtoFile(smt_file_path, smt_encoding)
    return smt_encoding

def findNearestEdgePoint(edge, point):
    min_dist = 10000
    index, nearest_pt = None, None
    for pt in edge.points:
        x,y = pt
        center_x, center_y = point.x, point.y
        distance = math.sqrt(math.pow(center_x-x, 2)+math.pow(center_y-y,2))
        if distance < min_dist:
            min_dist = distance
            nearest_pt = pt
    return nearest_pt

def findClosestPoint(elems, edge, point, debug=False):
    import shapely.geometry

    if debug:
        print("findClosestPoint() elems: ", elems)
        print("elems.is_empty: ", elems.is_empty)

    if elems.is_empty:
        # pick the edge point nearest to the given point
        nearest_pt = findNearestEdgePoint(edge, point)
        return shapely.geometry.Point(nearest_pt)
    elif isinstance(elems, shapely.geometry.MultiPoint):
        multiPts = list(elems.geoms)
    elif isinstance(elems, shapely.geometry.LineString):
        multiPts = [shapely.geometry.Point(*pt) for pt in elems.coords]
    else:
        raise NotImplementedError

    dist = []
    for pt in multiPts:
        dist.append(pt.distance(point))
    pt = multiPts[dist.index(min(dist))]
    return (pt.x, pt.y)


def isConditioned(obj):
	return obj is not obj._conditioned

# def resetConditionedVar(obj):
#     if not isinstance(obj, Samplable):
#         return None
#     obj._conditioned = obj
#     if (obj._dependencies is None):
#         return None
    
#     for dep in obj._dependencies:
#         resetConditionedVar(dep)
#     return None

# def resetConditionedObj(scenario):
#     for obj in scenario.objects:
#         resetConditionedVar(obj.position)
#         resetConditionedVar(obj.heading)

## Misc
def dependencies(thing):
	"""Dependencies which must be sampled before this value."""
	return getattr(thing, '_dependencies', ())

def needsSampling(thing):
	"""Whether this value requires sampling."""
	return isinstance(thing, Distribution) or dependencies(thing)

def supportInterval(thing):
	"""Lower and upper bounds on this value, if known."""
	if hasattr(thing, 'supportInterval'):
		return thing.supportInterval()
	elif isinstance(thing, (int, float)):
		return thing, thing
	else:
		return None, None

def underlyingFunction(thing):
	"""Original function underlying a distribution wrapper."""
	func = getattr(thing, '__wrapped__', thing)
	return getattr(func, '__func__', func)

def canUnpackDistributions(func):
	"""Whether the function supports iterable unpacking of distributions."""
	return getattr(func, '_canUnpackDistributions', False)

def unpacksDistributions(func):
	"""Decorator indicating the function supports iterable unpacking of distributions."""
	func._canUnpackDistributions = True
	return func

class RejectionException(Exception):
	"""Exception used to signal that the sample currently being generated must be rejected."""
	pass

## Abstract distributions

class DefaultIdentityDict:
	"""Dictionary which is the identity map by default.

	The map works on all objects, even unhashable ones, but doesn't support all
	of the standard mapping operations.
	"""
	def __init__(self):
		self.storage = {}

	def __getitem__(self, key):
		return self.storage.get(id(key), key)

	def __setitem__(self, key, value):
		self.storage[id(key)] = value

	def __contains__(self, key):
		return id(key) in self.storage

class Samplable(LazilyEvaluable):
	"""Abstract class for values which can be sampled, possibly depending on other values.

	Samplables may specify a proxy object 'self._conditioned' which must have the same
	distribution as the original after conditioning on the scenario's requirements. This
	allows transparent conditioning without modifying Samplable fields of immutable objects.
	"""
	def __init__(self, dependencies):
		deps = []
		props = set()
		for dep in dependencies:
			if needsSampling(dep) or needsLazyEvaluation(dep):
				deps.append(dep)
				props.update(requiredProperties(dep))
		super().__init__(props)
		self._dependencies = tuple(deps)	# fixed order for reproducibility
		self._conditioned = self	# version (partially) conditioned on requirements
		self.samplesRegion = False

	@staticmethod
	def sampleAll(quantities):
		"""Sample all the given Samplables, which may have dependencies in common.

		Reproducibility note: the order in which the quantities are given can affect the
		order in which calls to random are made, affecting the final result.
		"""
		subsamples = DefaultIdentityDict()
		for q in quantities:
			if q not in subsamples:
				subsamples[q] = q.sample(subsamples) if isinstance(q, Samplable) else q
		return subsamples

	def sample(self, subsamples=None):
		"""Sample this value, optionally given some values already sampled."""
		if subsamples is None:
			subsamples = DefaultIdentityDict()

		for child in self._conditioned._dependencies:
			if child not in subsamples:
				subsamples[child] = child.sample(subsamples)

		return self._conditioned.sampleGiven(subsamples)

	def sampleGiven(self, value):
		"""Sample this value, given values for all its dependencies.

		The default implementation simply returns a dictionary of dependency values.
		Subclasses must override this method to specify how actual sampling is done.
		"""
		return DefaultIdentityDict({ dep: value[dep] for dep in self._dependencies })

	def conditionTo(self, value):
		"""Condition this value to another value with the same conditional distribution."""
		# assert isinstance(value, Samplable)
		self._conditioned = value

	def evaluateIn(self, context):
		"""See LazilyEvaluable.evaluateIn."""
		value = super().evaluateIn(context)
		# Check that all dependencies have been evaluated
		assert all(not needsLazyEvaluation(dep) for dep in value._dependencies)
		return value

	def dependencyTree(self):
		"""Debugging method to print the dependency tree of a Samplable."""
		l = [str(self)]
		for dep in dependencies(self):
			for line in dep.dependencyTree():
				l.append('  ' + line)
		return l

class Constant(Samplable):
	def __init__(self, value):
		self._conditioned = value
		self._dependencies = tuple()
		self.value = value

	def sampleGiven(self, value):
		return self.value

class Distribution(Samplable):
	"""Abstract class for distributions."""

	defaultValueType = object

	def __new__(cls, *args, **kwargs):
		dist = super().__new__(cls)
		# at runtime, return a sample from the distribution immediately
		import scenic.syntax.veneer as veneer
		if veneer.simulationInProgress():
			dist.__init__(*args, **kwargs)
			return dist.sample()
		else:
			return dist

	def __init__(self, *dependencies, valueType=None):
		super().__init__(dependencies)
		if valueType is None:
			valueType = self.defaultValueType
		self.valueType = valueType

	def clone(self):
		"""Construct an independent copy of this Distribution."""
		raise NotImplementedError('clone() not supported by this distribution')

	@property
	@cached
	def isPrimitive(self):
		"""Whether this is a primitive Distribution."""
		try:
			self.clone()
			return True
		except NotImplementedError:
			return False

	def bucket(self, buckets=None):
		"""Construct a bucketed approximation of this Distribution.

		This function factors a given Distribution into a discrete distribution over
		buckets together with a distribution for each bucket. The argument *buckets*
		controls how many buckets the domain of the original Distribution is split into.
		Since the result is an independent distribution, the original must support
		clone().
		"""
		raise NotImplementedError('bucket() not supported by this distribution')

	def supportInterval(self):
		"""Compute lower and upper bounds on the value of this Distribution."""
		return None, None

	def __getattr__(self, name):
		if name.startswith('__') and name.endswith('__'):	# ignore special attributes
			return object.__getattribute__(self, name)
		return AttributeDistribution(name, self)

	def __call__(self, *args):
		return OperatorDistribution('__call__', self, args)

	def __iter__(self):
		raise TypeError(f'distribution {self} is not iterable')

	def _comparisonError(self, other):
		raise RuntimeParseError('random values cannot be compared '
		                        '(and control flow cannot depend on them)')

	__lt__ = _comparisonError
	__le__ = _comparisonError
	__gt__ = _comparisonError
	__ge__ = _comparisonError
	__eq__ = _comparisonError
	__ne__ = _comparisonError

	def __hash__(self):		# need to explicitly define since we overrode __eq__
		return id(self)

	def __len__(self):
		raise RuntimeParseError('cannot take the len of a random value')

	def __bool__(self):
		raise RuntimeParseError('control flow cannot depend on a random value')

## Derived distributions

class CustomDistribution(Distribution):
	"""Distribution with a custom sampler given by an arbitrary function"""
	def __init__(self, sampler, *dependencies, name='CustomDistribution', evaluator=None):
		super().__init__(*dependencies)
		self.sampler = sampler
		self.name = name
		self.evaluator = evaluator

	def sampleGiven(self, value):
		return self.sampler(value)

	def evaluateInner(self, context):
		if self.evaluator is None:
			raise NotImplementedError('evaluateIn() not supported by this distribution')
		return self.evaluator(self, context)

	def isEquivalentTo(self, other):
		if not type(other) is CustomDistribution:
			return False
		return (self.sampler == other.sampler
			and self.name == other.name
			and self.evaluator == other.evaluator)

	def __str__(self):
		return f'{self.name}{argsToString(self.dependencies)}'

class TupleDistribution(Distribution, collections.abc.Sequence):
	"""Distributions over tuples (or namedtuples, or lists)."""
	def __init__(self, *coordinates, builder=tuple):
		super().__init__(*coordinates)
		self.coordinates = coordinates
		self.builder = builder

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		raise NotImplementedError

	def __len__(self):
		return len(self.coordinates)

	def __getitem__(self, index):
		return self.coordinates[index]

	def __iter__(self):
		yield from self.coordinates

	def sampleGiven(self, value):
		return self.builder(value[coordinate] for coordinate in self.coordinates)

	def evaluateInner(self, context):
		coordinates = (valueInContext(coord, context) for coord in self.coordinates)
		return TupleDistribution(*coordinates, builder=self.builder)

	def isEquivalentTo(self, other):
		if not type(other) is TupleDistribution:
			return False
		return (areEquivalent(self.coordinates, other.coordinates)
			and self.builder == other.builder)

	def __str__(self):
		coords = ', '.join(str(c) for c in self.coordinates)
		return f'({coords}, builder={self.builder})'

def toDistribution(val):
	"""Wrap Python data types with Distributions, if necessary.

	For example, tuples containing Samplables need to be converted into TupleDistributions
	in order to keep track of dependencies properly.
	"""
	if isinstance(val, (tuple, list)):
		coords = [toDistribution(c) for c in val]
		if any(needsSampling(c) or needsLazyEvaluation(c) for c in coords):
			if isinstance(val, tuple) and hasattr(val, '_fields'):		# namedtuple
				builder = type(val)._make
			else:
				builder = type(val)
			return TupleDistribution(*coords, builder=builder)
	return val

class FunctionDistribution(Distribution):
	"""Distribution resulting from passing distributions to a function"""
	def __init__(self, func, args, kwargs, support=None, valueType=None):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if valueType is None:
			valueType = typing.get_type_hints(func).get('return')
		super().__init__(*args, *kwargs.values(), valueType=valueType)
		self.function = func
		self.arguments = args
		self.kwargs = kwargs
		self.support = support

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	for arg in self.arguments:
	# 		if isinstance(arg, Samplable) and not isConditioned(arg):
	# 			arg.conditionforSMT(condition, conditioned_bool)
	# 	for kwarg in self.kwargs:
	# 		if isinstance(kwarg, Samplable) and not isConditioned(kwarg):
	# 			kwarg.conditionforSMT(condition, conditioned_bool)
	# 	for support in self.support:
	# 		if isinstance(support, Samplable) and not isConditioned(support):
	# 			support.conditionforSMT(condition, conditioned_bool)
	# 	return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		"""to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		   encodeToSMT() must return 'cached_variables' dictionary
		"""
		# import scenic.core.geometry as geometry
		raise NotImplementedError

		if debug:
			print("FunctionDistribution")

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)

		if self in cached_variables.keys():
			if debug:
				print("FunctionDistribution already cached")
			return cached_variables[self]
			
		import scenic.core.vectors as vectors
		import scenic.core.geometry as geometry
		import scenic.core.regions as regions
		import scenic.domains.driving.roads as roads

		if self.function == vectors.OrientedVector.make:
			position = self.arguments[0]
			heading = self.arguments[1]
			output = vectors.OrientedVector.makeEncodeSMT(smt_file_path, cached_variables, position, heading, debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.function == geometry.sin:
			x = self.arguments[0]
			output = geometry.sinEncodeSMT(smt_file_path, cached_variables, x, debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.function == geometry.cos:
			x = self.arguments[0]
			output = geometry.cosEncodeSMT(smt_file_path, cached_variables, x, debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.function == geometry.normalizeAngle:
			angle = checkAndEncodeSMT(smt_file_path, cached_variables, self.arguments[0], debug=debug)
			output = normalizeAngle_SMT(smt_file_path, cached_variables, angle)
			return cacheVarName(cached_variables, self, output)

		# Issue: How to recover the road class from the function
		# elif self.function == roads.Road.sectionAt:
		# 	point = checkAndEncodeSMT(smt_file_path, cached_variables, self.arguments[0], debug=debug)
		# 	output = 
		else:
			raise NotImplementedError

		return None

	def sampleGiven(self, value):
		args = []
		for arg in self.arguments:
			if isinstance(arg, StarredDistribution):
				val = value[arg]
				try:
					iter(val)
				except TypeError:	# TODO improve backtrace
					raise TypeError(f"'{type(val).__name__}' object on line {arg.lineno} "
					                "is not iterable") from None
				args.extend(val)
			else:
				args.append(value[arg])
		kwargs = { name: value[arg] for name, arg in self.kwargs.items() }
		return self.function(*args, **kwargs)

	def evaluateInner(self, context):
		function = valueInContext(self.function, context)
		arguments = tuple(valueInContext(arg, context) for arg in self.arguments)
		kwargs = { name: valueInContext(arg, context) for name, arg in self.kwargs.items() }
		return FunctionDistribution(function, arguments, kwargs)

	def supportInterval(self):
		if self.support is None:
			return None, None
		subsupports = (supportInterval(arg) for arg in self.arguments)
		kwss = { name: supportInterval(arg) for name, arg in self.kwargs.items() }
		return self.support(*subsupports, **kwss)

	def isEquivalentTo(self, other):
		if not type(other) is FunctionDistribution:
			return False
		return (self.function == other.function
			and areEquivalent(self.arguments, other.arguments)
			and areEquivalent(self.kwargs, other.kwargs)
			and self.support == other.support)

	def __str__(self):
		args = argsToString(itertools.chain(self.arguments, self.kwargs.items()))
		return f'{self.function.__name__}{args}'

def distributionFunction(wrapped=None, *, support=None, valueType=None):
	"""Decorator for wrapping a function so that it can take distributions as arguments."""
	if wrapped is None:		# written without arguments as @distributionFunction
		return lambda wrapped: distributionFunction(wrapped,
		                                            support=support, valueType=valueType)

	@unpacksDistributions
	@wrapt.decorator
	def wrapper(wrapped, instance, args, kwargs):
		def helper(*args, **kwargs):
			args = tuple(toDistribution(arg) for arg in args)
			kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
			if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
				return FunctionDistribution(wrapped, args, kwargs, support, valueType)
			elif any(needsLazyEvaluation(arg)
			         for arg in itertools.chain(args, kwargs.values())):
				# recursively call this helper (not the original function), since the
				# delayed arguments may evaluate to distributions, in which case we'll
				# have to make a FunctionDistribution
				return makeDelayedFunctionCall(helper, args, kwargs)
			else:
				return wrapped(*args, **kwargs)
		return helper(*args, **kwargs)
	return wrapper(wrapped)

def monotonicDistributionFunction(method, valueType=None):
	"""Like distributionFunction, but additionally specifies that the function is monotonic."""
	def support(*subsupports, **kwss):
		mins, maxes = zip(*subsupports)
		kwmins = { name: interval[0] for name, interval in kwss.items() }
		kwmaxes = { name: interval[1] for name, interval in kwss.items() }
		l = None if None in mins or None in kwmins else method(*mins, **kwmins)
		r = None if None in maxes or None in kwmaxes else method(*maxes, **kwmaxes)
		return l, r
	return distributionFunction(method, support=support, valueType=valueType)

class StarredDistribution(Distribution):
	"""A placeholder for the iterable unpacking operator * applied to a distribution."""
	def __init__(self, value, lineno):
		assert isinstance(value, Distribution)
		self.value = value
		self.lineno = lineno	# for error handling when unpacking fails
		super().__init__(value, valueType=value.valueType)

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False, encode=True):
		if debug:
			print("Class StarredDistribution encodeToSMT")

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)

		if self in cached_variables.keys():
			if debug:
				print("StarredDistribution already cached")
			return cached_variables[self]

		return self.value._conditioned.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=encode)	

	def sampleGiven(self, value):
		return value[self.value]

	def evaluateInner(self, context):
		return StarredDistribution(valueInContext(self.value, context))

	def __str__(self):
		return f'*{self.value}'

class MethodDistribution(Distribution):
	"""Distribution resulting from passing distributions to a method of a fixed object"""
	def __init__(self, method, obj, args, kwargs, valueType=None):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if valueType is None:
			valueType = typing.get_type_hints(method).get('return')
		super().__init__(*args, *kwargs.values(), valueType=valueType)
		self.method = method
		self.object = obj
		self.arguments = args
		self.kwargs = kwargs

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False, encode=True):
		"""to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		   encodeToSMT() must return 'cached_variables' dictionary
		"""
		# import scenic.core.geometry as geometry
		if debug:
			print("MethodDistribution")
			print("self.method: ", self.method)
			print("type(self.object): ", type(self.object))
			for arg in self.arguments:
				print("type(arg): ", type(arg))

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)

		if self in set(cached_variables.keys()):
			if debug:
				print( "MethodDistribution already exists in cached_variables dict")
			return cached_variables[self]

		import scenic.domains.driving.roads as roads
		import scenic.core.vectors as vectors
		import scenic.core.type_support as type_support
		obj = None
		if isinstance(self.object, Samplable) and isConditioned(self.object):
			obj = self.object._conditioned
		# elif isinstance(self.object, Options):
		# 	obj = self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=False)
		elif isinstance(self.object, type_support.TypecheckedDistribution):
			obj = self.object.dist
		else:
			obj = self.object            
		
		output = None
		if self.method == roads.Network.findPointIn:
			assert(len(self.arguments)==2)
			point = self.arguments[0]
			elems = self.arguments[1]
			output = obj.findPointInEncodeSMT(smt_file_path, cached_variables, point, elems, debug=debug)

		elif self.method == roads.Network.elementAt:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			output = obj.elementAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == roads.Network.roadAt:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			output = obj.roadAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == roads.Network.laneAt:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			output = obj.laneAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == roads.Network.laneSectionAtEncodeSMT:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			output = obj.laneSectionAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == roads.Network.laneGroupAt:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			if not encode and isConditioned(self.arguments[0]) and isinstance(self.arguments[0]._conditioned, vectors.Vector):
				import scenic.domains.driving.roads as roads
				if isinstance(self.object, roads.Network):
					lanegroup = self.object.laneGroupAt(self.arguments[0]._conditioned)
					if debug:
						print("MethodDistribution method: laneGroupAt")
						plt.plot(*lanegroup.polygon.exterior.xy, color='r')
						plt.show()
					return lanegroup
			else:
				outputt = obj.laneGroupAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == roads.Network.crossingAt:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			output = obj.crossingAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == roads.Network.intersectionAt:
			assert(len(self.arguments)==1)
			point = self.arguments[0]
			output = obj.intersectionAtEncodeSMT(smt_file_path, cached_variables, point, debug=debug)

		elif self.method == vectors.VectorField.__getitem__:
			assert(len(self.arguments)==1)
			if debug:
				print("type(self.arguments[0]): ", type(self.arguments[0]))
				print("self.arguments[0]: ", self.arguments[0])

			if isConditioned(self.arguments[0]) and isinstance(self.arguments[0]._conditioned, vectors.Vector):
				vector = self.arguments[0]._conditioned
				output = str(self.method(self.object, vector))
				if debug:
					print("MethodDistribution self.method == vectors.VectorField.__getitem__")
					print("output: ", output)
				return cacheVarName(cached_variables, self, output)

			optionsRegion = self.arguments[0].encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=False)
			if debug:
				print("type(self.arguments[0]): ", type(self.arguments[0]))
				print("self.arguments[0]: ", self.arguments[0])
				print( "operatordist optionRegion: "+str(optionsRegion))

			assert(isinstance(optionsRegion, Options))

			import scenic.core.vectors as vectors
			if isConditioned(self.arguments[0]) and isinstance(self.arguments[0]._conditioned, vectors.Vector):
				vector = self.arguments[0]._conditioned
				for region in optionsRegion.options:
					if region.containsPoint(vector):
						heading = str(region.nominalDirectionsAt(vector))
						writeSMTtoFile(smt_file_path, smt_assert("equal", heading, output))
						return cacheVarName(cached_variables, self, output)
				return None

			if self.object in cached_variables.keys():
				x, y = self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug)
				if debug:
					print("self.object already cached: ", (x,y))
			else:
				x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
				y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
			smt_var = (x,y)
			output = findVariableName(smt_file_path, cached_variables, 'methodDist', debug=debug)

			import scenic.domains.driving.roads as roads
			import shapely.geometry
			heading_var = output
			if optionsRegion.checkOptionsType(roads.NetworkElement):
				encodeHeading(cached_variables, optionsRegion.options, smt_file_path, smt_var, heading_var, debug=debug)
			elif optionsRegion.checkOptionsType(shapely.geometry.LineString):
				encodePolyLineHeading(cached_variables, optionsRegion.options, smt_file_path, smt_var, heading_var, debug=debug)
			else:
				raise NotImplementedError

		else:
			raise NotImplementedError

		return cacheVarName(cached_variables, self, output)

	def sampleGiven(self, value):
		args = []
		for arg in self.arguments:
			if isinstance(arg, StarredDistribution):
				args.extend(value[arg.value])
			else:
				args.append(value[arg])
		kwargs = { name: value[arg] for name, arg in self.kwargs.items() }
		return self.method(self.object, *args, **kwargs)

	def evaluateInner(self, context):
		obj = valueInContext(self.object, context)
		arguments = tuple(valueInContext(arg, context) for arg in self.arguments)
		kwargs = { name: valueInContext(arg, context) for name, arg in self.kwargs.items() }
		return MethodDistribution(self.method, obj, arguments, kwargs)

	def isEquivalentTo(self, other):
		if not type(other) is MethodDistribution:
			return False
		return (self.method == other.method
			and areEquivalent(self.object, other.object)
			and areEquivalent(self.arguments, other.arguments)
			and areEquivalent(self.kwargs, other.kwargs))

	def __str__(self):
		args = argsToString(itertools.chain(self.arguments, self.kwargs.items()))
		return f'{self.object}.{self.method.__name__}{args}'

def distributionMethod(method):
	"""Decorator for wrapping a method so that it can take distributions as arguments."""
	@unpacksDistributions
	@wrapt.decorator
	def wrapper(wrapped, instance, args, kwargs):
		def helper(*args, **kwargs):
			args = tuple(toDistribution(arg) for arg in args)
			kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
			if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
				return MethodDistribution(method, instance, args, kwargs)
			elif any(needsLazyEvaluation(arg)
			         for arg in itertools.chain(args, kwargs.values())):
				# see analogous comment in distributionFunction
				return makeDelayedFunctionCall(helper, args, kwargs)
			else:
				return wrapped(*args, **kwargs)
		return helper(*args, **kwargs)
	return wrapper(method)

class AttributeDistribution(Distribution):
	"""Distribution resulting from accessing an attribute of a distribution"""
	def __init__(self, attribute, obj):
		super().__init__(obj)
		self.attribute = attribute
		self.object = obj

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False, encode=True):
		if debug:
			print( "Class AttributeDistribution encodeToSMT")
			print("self: ", self)
			print("encode: ", encode)

		if encode and isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)
		
		if encode and self in cached_variables.keys():
			print( "Class AttributeDistribution self in cached_variables")

		from scenic.core.object_types import Point
		import scenic.domains.driving.roads as roads
		import scenic.core.type_support as type_support

		if isinstance(self.object, type_support.TypecheckedDistribution):
			obj = self.object.dist
		else:
			obj = self.object

		output_smt = None
		if encode:
			if isinstance(obj, Point):
				output_smt = getattr(obj._conditioned, self.attribute).encodeToSMT(smt_file_path, 
										cached_variables, debug=debug)

			elif isinstance(obj, Options):
				if obj.checkOptionsType(roads.NetworkElement):
					return obj.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=encode)
				else: 
					raise NotImplementedError
			elif isinstance(obj, UniformDistribution):
				return obj.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=encode)
			else:
				print("NEW CASE: ", type(obj._conditioned))
				print("NEW CASE: AttributeDistribution type(self.object): ", type(obj))
				print("NEW CASE: AttributeDistribution self.object: ", obj)
				print("NEW CASE: AttributeDistribution self.attribute: ", self.attribute)
				raise NotImplementedError

		else:
			if isinstance(obj, Options):
				if debug:
					print("AttributeDistribution encode=False, type(obj) is Options")
				if obj.checkOptionsType(roads.NetworkElement):
					networkObjs = obj.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=encode)
					if debug:
						print("networkObjs: ", networkObjs)

					if obj in cached_variables.keys():
						output_smt = cached_variables[obj]
						cacheVarName(cached_variables, self, output_smt)
					return networkObjs
				else: 
					print("NEW CASE: AttributeDistribution type(self.object): ", type(obj))
					print("NEW CASE: AttributeDistribution self.object: ", obj)
					print("NEW CASE: AttributeDistribution self.attribute: ", self.attribute)
					raise NotImplementedError

			elif isinstance(obj, UniformDistribution):
				networkObj = obj.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=encode)
				if networkObj is None:
					return None
				if obj in cached_variables.keys():
					output_smt = cached_variables[obj]
					cacheVarName(cached_variables, self, output_smt)
				return Options(outputObjs)

			elif isinstance(obj, OperatorDistribution):
				import scenic.core.vectors as vectors
				if self.attribute == 'orientation':
					if debug:
						print("AttributeDistribution obj: ", obj)
						print("AttributeDistribution obj._conditioned: ", obj._conditioned)
					regionOptions = obj.encodeToSMT(smt_file_path, cached_variables, debug, encode=False)
					if debug:
						print("AttributeDistribution obj OperatorDistribution regionOptions: ", regionOptions)
					assert(isinstance(regionOptions, Options))

					if isinstance(obj._conditioned, vectors.Vector):
						vector = obj._conditioned
						possibleRegions = []
						import scenic.domains.driving.roads as roads
						for reg in regionOptions.options:
							if isinstance(reg, roads.NetworkElement) and reg.containsPoint(vector):
								possibleRegions.append(reg)
						if possibleRegions == []:
							return None

						if obj in cached_variables.keys():
							output_smt = cached_variables[obj]
							cacheVarName(cached_variables, self, output_smt)
						return Options(possibleRegions)
					else:
						if obj in cached_variables.keys():
							output_smt = cached_variables[obj]
							cacheVarName(cached_variables, self, output_smt)
						return regionOptions
				else:
					print("NEW CASE: AttributeDistribution type(self.object): ", type(obj))
					print("NEW CASE: AttributeDistribution self.object: ", obj)
					print("NEW CASE: AttributeDistribution self.attribute: ", self.attribute)
					raise NotImplementedError

			elif isinstance(obj, AttributeDistribution):
				if self.attribute == 'intersect':
					possibleRegions = obj.encodeToSMT(smt_file_path, cached_variables, debug, encode=False)
					assert(isinstance(possibleRegions, Options))
					return possibleRegions
				elif self.attribute == '_opposite':
					possibleRegions = obj.encodeToSMT(smt_file_path, cached_variables, debug, encode=False)
					if isinstance(possibleRegions, Options):
						pass
					else:
						outputRegion = possibleRegions._opposite
						if debug:
							print("AttributeDistribution attribute: _opposite")
							plt.plot(*outputRegion.exterior.xy, color='r')
							plt.show()
						return outputRegion
				elif self.attribute == 'lanes':
					possibleRegions = obj.encodeToSMT(smt_file_path, cached_variables, debug, encode=False)
					if isinstance(possibleRegions, Options):
						pass
					else:
						outputRegions = possibleRegions.lanes
						if debug:
							print("AttributeDistribution attribute: lanes")
							for lane in outputRegions:
								plt.plot(*lane.polygon.exterior.xy, color='g')
								plt.show()
						return outputRegions
				else:
					print("NEW CASE: AttributeDistribution type(self.object): ", type(obj))
					print("NEW CASE: AttributeDistribution self.object: ", obj)
					print("NEW CASE: AttributeDistribution self.attribute: ", self.attribute)
			else:
				print("NEW CASE: AttributeDistribution type(self.object): ", type(obj))
				print("NEW CASE: AttributeDistribution self.object: ", obj)
				print("NEW CASE: AttributeDistribution self.attribute: ", self.attribute)
				raise NotImplementedError

		return cacheVarName(cached_variables, self, output_smt)

	def sampleGiven(self, value):
		obj = value[self.object]
		return getattr(obj, self.attribute)

	def evaluateInner(self, context):
		obj = valueInContext(self.object, context)
		return AttributeDistribution(self.attribute, obj)

	def supportInterval(self):
		obj = self.object
		if isinstance(obj, Options):
			attrs = (getattr(opt, self.attribute) for opt in obj.options)
			mins, maxes = zip(*(supportInterval(attr) for attr in attrs))
			l = None if any(sl is None for sl in mins) else min(mins)
			r = None if any(sr is None for sr in maxes) else max(maxes)
			return l, r
		return None, None

	def isEquivalentTo(self, other):
		if not type(other) is AttributeDistribution:
			return False
		return (self.attribute == other.attribute
			and areEquivalent(self.object, other.object))

	def __call__(self, *args):
		vty = self.object.valueType
		if vty is not object and (func := getattr(vty, self.attribute, None)):
			if isinstance(func, property):
				func = func.fget
			retTy = typing.get_type_hints(func).get('return')
		else:
			retTy = None
		return OperatorDistribution('__call__', self, args, valueType=retTy)

	def __str__(self):
		return f'{self.object}.{self.attribute}'

class OperatorDistribution(Distribution):
	"""Distribution resulting from applying an operator to one or more distributions"""
	def __init__(self, operator, obj, operands, valueType=None):
		operands = tuple(toDistribution(arg) for arg in operands)
		if valueType is None:
			valueType = self.inferType(obj, operator)
		super().__init__(obj, *operands, valueType=valueType)
		self.operator = operator
		self.object = obj
		self.operands = operands

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	if isinstance(self.object._conditioned, Samplable) and not isConditioned(self.object._conditioned):
	# 		self.object._conditioned.conditionforSMT(condition, conditioned_bool)
	# 	for op in self.operands:
	# 		if isinstance(op._conditioned, Samplable) and not isConditioned(op._conditioned):
	# 			op._conditioned.conditionforSMT(condition, conditioned_bool)
	# 	return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False, encode = True):
		"""to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		   encodeToSMT() must return 'cached_variables' dictionary
		"""
		if debug:
			print( "OperatorDistribution")
			print( "operator: "+self.operator)
			print( "type(object): "+str(type(self.object._conditioned)))
			for op in self.operands:
				print( "type(operand): "+str(type(op)))

		if encode and isConditioned(self) and not isinstance(self._conditioned, Samplable):
			if debug:
				print( "OperatorDist is conditioned")
			return cacheVarName(cached_variables, self, self._conditioned)

		if encode and self in cached_variables.keys():
			if debug:
				print( "OperatorDistribution already exists in cached_variables dict: ", cached_variables[self])
			return cached_variables[self]

		output = findVariableName(smt_file_path, cached_variables, 'opdist', debug=debug)

		if self.operator in ['__add__', '__radd__' , '__sub__', '__rsub__', '__truediv__', '__rtruediv__', '__mul__', '__rmul__',\
			'__floordiv__', '__rfloordiv__','__mod__', '__rmod__','__divmod__', '__rdivmod__','__pow__', '__rpow__']:

			obj_var = self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug)
			assert(len(self.operands) < 2)
			operand = self.operands[0]
			operand_smt = checkAndEncodeSMT(smt_file_path, cached_variables, operand, debug=debug)

			if self.operator == '__add__' or self.operator == '__radd__':
				summation = smt_add(obj_var, operand_smt)
				smt_encoding = smt_assert("equal", output, summation)

			elif self.operator == '__mul__' or self.operator == '__rmul__':
				multiplication = smt_multiply(obj_var, operand_smt)
				smt_encoding = smt_assert("equal", output, multiplication)

			elif self.operator == '__sub__':
				subtraction = smt_subtract(obj_var, operand_smt)
				smt_encoding = smt_assert("equal", output, subtraction)

			elif self.operator == '__rsub__':
				subtraction = smt_subtract(operand_smt, obj_var)
				smt_encoding = smt_assert("equal", output, subtraction)

			elif self.operator == '__truediv__':
				truediv = smt_divide(obj_var, operand_smt)
				smt_encoding = smt_assert("equal", output, truediv)

			elif self.operator == '__rtruediv__':
				division = smt_divide(operand_smt, obj_var)
				smt_encoding = smt_assert("equal", output, division)

			# elif self.operator == '__mod__':
			# 	modular = smt_mod(obj_var, operand_smt)
			# 	smt_encoding = smt_assert("equal", output, modular)

			# elif self.operator == '__rmod__':
			# 	modular = smt_mod(operand_smt, obj_var)
			# 	smt_encoding = smt_assert("equal", output, modular)

			else:# TODO: floordiv, rfloordiv, divmod, rdivmod, pow, rpow
				raise NotImplementedError

			writeSMTtoFile(smt_file_path, smt_encoding)

		elif self.operator == '__call__':
			if isinstance(self.object, AttributeDistribution) and self.object.attribute == 'intersect':
				assert(len(self.operands)==1)

				distOverRegions = self.object.encodeToSMT(smt_file_path, cached_variables, debug, encode=False)
				if debug:
					print("OperatorDistribution __call__ distOverRegions: ", distOverRegions)
				if distOverRegions is None:
					return None
				assert(isinstance(distOverRegions, Options))
				possibleRegions = []
				otherRegion = self.operands[0]

				import scenic.core.regions as regions
				import shapely.geometry.polygon as polygon
				assert(isinstance(otherRegion, regions.SectorRegion))
				if debug:
					print("OperatorDistribution __call__ otherRegion: ", otherRegion)

				if otherRegion.checkRandomVar():
					for reg in distOverRegions.options:
						regPolygon = reg.polygon
						intersection = regPolygon.intersection(otherRegion.polygon)
						if not intersection.is_empty:
							if encode:
								possibleRegions.append(intersection)
							else: 
								possibleRegions.append(reg)

				else:
					regionAroundEgo = cached_variables['regionAroundEgo_polygon']
					for reg in distOverRegions.options:
						regPolygon = reg.polygon
						intersection = regionAroundEgo.intersection(regPolygon)
						if not intersection.is_empty:
							if encode:
								possibleRegions.append(intersection)
							else: 
								possibleRegions.append(reg)

				# possibleRegions.append(self.operands[0])

				if not encode:
					return Options(possibleRegions)

				if self.object in cached_variables.keys():
					x, y = self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug)
					if debug:
						print("self.object already cached: ", (x,y))
				else:
					x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
					y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
				output = (x,y)

				import shapely.geometry.multipolygon
				polygonalRegions = []
				for elem in possibleRegions:
					if isinstance(elem, shapely.geometry.multipolygon.MultiPolygon):
						for geom in elem.geoms:
							polygonalRegions.append(geom)
					elif isinstance(elem, shapely.geometry.polygon.Polygon):
						polygonalRegions.append(elem)
					else:
						print("elem: ", elem)
						raise NotImplementedError

				if debug:
					print("in Options class, polygonalRegions: ", polygonalRegions)

				multipolygon = shapely.geometry.multipolygon.MultiPolygon(polygonalRegions)
				polygonReg = regions.regionFromShapelyObject(multipolygon)
				polygonReg.encodeToSMT(smt_file_path, cached_variables, output, debug=debug)

		elif self.operator == '__getitem__': # called only from VectorField and Vector Classes
			import scenic.core.vectors as vectors
			if isinstance(self.object, AttributeDistribution) and self.object.attribute == 'orientation':
				# type(self.object) could be AttributeDist, Options
				import scenic.core.vectors as vectors

				if debug:
					print( "OperatorDistribution self.operator == __getitem__")

				# all 'encode=False' flag outputs either Options class or heading
				optionsRegion = self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=False)
				assert(isinstance(optionsRegion, Options))

				if debug:
					print( "operatordist optionRegion: "+str(optionsRegion))

				import scenic.core.vectors as vectors
				if isConditioned(self.operands[0]) and isinstance(self.operands[0]._conditioned, vectors.Vector):
					vector = self.operands[0]._conditioned
					for region in optionsRegion.options:
						if region.containsPoint(vector):
							heading = region.nominalDirectionsAt(vector)
							if debug:
								print("OperatorDistribution __getitem__ heading: ", heading)
							if isinstance(heading, tuple):
								heading_smt = str(heading[0])
							elif isinstance(heading, (float, int)):
								heading_smt = str(heading)
							else:
								raise NotImplementedError
							writeSMTtoFile(smt_file_path, smt_assert("equal", heading_smt, output))
							return cacheVarName(cached_variables, self, output)
					return None

				if self.object in cached_variables.keys():
					x, y = cached_variables[self.object]
					if debug:
						print("self.object already cached: ", (x,y))
				else:
					x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
					y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
				smt_var = (x,y)
				output = findVariableName(smt_file_path, cached_variables, 'opdist', debug=debug)

				import scenic.domains.driving.roads as roads
				assert(optionsRegion.checkOptionsType(roads.NetworkElement))
				heading_var = output
				encodeHeading(cached_variables, optionsRegion.options, smt_file_path, smt_var, heading_var, debug=debug)

			else:
				raise NotImplementedError

		elif self.operator == 'angleTo':
			import scenic.core.vectors as vectors
			if debug:
				print("OperatorDist operator angleTo case")
			# self_vector_smt = self.object.encodeToSMT(smt_file_path, cached_variables, debug)
			# other_vector_smt= self.operands[0].encodeToSMT(smt_file_path, cached_variables, debug)
			# if debug:
			# 	print("self_vector_smt: ", self_vector_smt)
			# 	print("other_vector_smt: ", other_vector_smt)
			# self_vector = vectors.Vector(self_vector_smt[0], self_vector_smt[1])
			# other_vector = vectors.Vector(other_vector_smt[0], other_vector_smt[1])
			# output = self_vector.angleToEncodeSMT(smt_file_path, cached_variables, other_vector_smt, debug)
			assert(isConditioned(self.object) and isConditioned(self.operands[0]))
			angle = self.object._conditioned.angleTo(self.operands[0]._conditioned)
			writeSMTtoFile(smt_file_path, smt_assert("equal", output, str(angle)))

		else:
			print("self.operator: " + str(self.operator))
			if debug:
				print( "self.operator: " + str(self.operator))
			raise NotImplementedError

		return cacheVarName(cached_variables, self, output)


	@staticmethod
	def inferType(obj, operator):
		if issubclass(obj.valueType, (float, int)):
			return float
		return None

	def sampleGiven(self, value):
		first = value[self.object]
		rest = [value[child] for child in self.operands]
		op = getattr(first, self.operator)
		result = op(*rest)
		# handle horrible int/float mismatch
		# TODO what is the right way to fix this???
		if result is NotImplemented and isinstance(first, int):
			first = float(first)
			op = getattr(first, self.operator)
			result = op(*rest)
		return result

	def evaluateInner(self, context):
		obj = valueInContext(self.object, context)
		operands = tuple(valueInContext(arg, context) for arg in self.operands)
		return OperatorDistribution(self.operator, obj, operands)

	def supportInterval(self):
		if self.operator in ('__add__', '__radd__', '__sub__', '__rsub__', '__truediv__'):
			assert len(self.operands) == 1
			l1, r1 = supportInterval(self.object)
			l2, r2 = supportInterval(self.operands[0])
			if l1 is None or l2 is None or r1 is None or r2 is None:
				return None, None
			if self.operator == '__add__' or self.operator == '__radd__':
				l = l1 + l2
				r = r1 + r2
			elif self.operator == '__sub__':
				l = l1 - r2
				r = r1 - l2
			elif self.operator == '__rsub__':
				l = l2 - r1
				r = r2 - l1
			elif self.operator == '__truediv__':
				if l2 > 0:
					l = l1 / r2 if l1 >= 0 else l1 / l2
					r = r1 / l2 if r1 >= 0 else r1 / r2
				else:
					l, r = None, None 	# TODO improve
			return l, r
		return None, None

	def isEquivalentTo(self, other):
		if not type(other) is OperatorDistribution:
			return False
		return (self.operator == other.operator
			and areEquivalent(self.object, other.object)
			and areEquivalent(self.operands, other.operands))

	def __str__(self):
		return f'{self.object}.{self.operator}{argsToString(self.operands)}'

# Operators which can be applied to distributions.
# Note that we deliberately do not include comparisons and __bool__,
# since Scenic does not allow control flow to depend on random variables.
allowedOperators = (
	'__neg__',
	'__pos__',
	'__abs__',
	'__add__', '__radd__',
	'__sub__', '__rsub__',
	'__mul__', '__rmul__',
	'__truediv__', '__rtruediv__',
	'__floordiv__', '__rfloordiv__',
	'__mod__', '__rmod__',
	'__divmod__', '__rdivmod__',
	'__pow__', '__rpow__',
	'__round__',
	'__getitem__',
)
def makeOperatorHandler(op):
	def handler(self, *args):
		return OperatorDistribution(op, self, args)
	return handler
for op in allowedOperators:
	setattr(Distribution, op, makeOperatorHandler(op))

import scenic.core.type_support as type_support

class MultiplexerDistribution(Distribution):
	"""Distribution selecting among values based on another distribution."""

	def __init__(self, index, options):
		self.index = index
		self.options = tuple(toDistribution(opt) for opt in options)
		assert len(self.options) > 0
		valueType = type_support.unifyingType(self.options)
		super().__init__(index, *self.options, valueType=valueType)

	def sampleGiven(self, value):
		idx = value[self.index]
		assert 0 <= idx < len(self.options), (idx, len(self.options))
		return value[self.options[idx]]

	def evaluateInner(self, context):
		return type(self)(valueInContext(self.index, context),
		                  (valueInContext(opt, context) for opt in self.options))

	def isEquivalentTo(self, other):
		if not type(other) == type(self):
			return False
		return (areEquivalent(self.index, other.index)
		        and areEquivalent(self.options, other.options))

## Simple distributions

class Range(Distribution):
	"""Uniform distribution over a range"""
	def __init__(self, low, high):
		low = type_support.toScalar(low, f'Range endpoint {low} is not a scalar')
		high = type_support.toScalar(high, f'Range endpoint {high} is not a scalar')
		super().__init__(low, high, valueType=float)
		self.low = low
		self.high = high

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	if isinstance(self.low, Samplable) and not isConditioned(self.low):
	# 		self.low.conditionforSMT(condition, conditioned_bool)
	# 	if isinstance(self.high, Samplable) and not isConditioned(self.high):
	# 		self.high.conditionforSMT(condition, conditioned_bool)
	# 	return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		"""
			smt_file_path must be an absolute path, not relative to a root of non-home folder
			to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		"""
		if debug:
			print("Range Class")

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)
		
		if self in cached_variables.keys():
			if debug:
				print( "Range already cached")
			return cached_variables[self]

		low = checkAndEncodeSMT(smt_file_path, cached_variables, self.low, debug=debug)
		high = checkAndEncodeSMT(smt_file_path, cached_variables, self.high, debug=debug)
		var_name = findVariableName(smt_file_path, cached_variables, 'range', debug=debug)

		lower_bound = smt_lessThanEq(low, var_name)
		upper_bound = smt_lessThanEq(var_name, high)
		smt_encoding = smt_assert("and", lower_bound, upper_bound)
		writeSMTtoFile(smt_file_path, smt_encoding)

		return cacheVarName(cached_variables, self, var_name)

	def __contains__(self, obj):
		return self.low <= obj and obj <= self.high

	def clone(self):
		return type(self)(self.low, self.high)

	def bucket(self, buckets=None):
		if buckets is None:
			buckets = 5
		if not isinstance(buckets, int) or buckets < 1:
			raise RuntimeError(f'Invalid buckets for Range.bucket: {buckets}')
		if not isinstance(self.low, float) or not isinstance(self.high, float):
			raise RuntimeError(f'Cannot bucket Range with non-constant endpoints')
		endpoints = numpy.linspace(self.low, self.high, buckets+1)
		ranges = []
		for i, left in enumerate(endpoints[:-1]):
			right = endpoints[i+1]
			ranges.append(Range(left, right))
		return Options(ranges)

	def sampleGiven(self, value):
		return random.uniform(value[self.low], value[self.high])

	def evaluateInner(self, context):
		low = valueInContext(self.low, context)
		high = valueInContext(self.high, context)
		return Range(low, high)

	def isEquivalentTo(self, other):
		if not type(other) is Range:
			return False
		return (areEquivalent(self.low, other.low)
			and areEquivalent(self.high, other.high))

	def __str__(self):
		return f'Range({self.low}, {self.high})'

class Normal(Distribution):
	"""Normal distribution"""
	def __init__(self, mean, stddev):
		mean = type_support.toScalar(mean, f'Normal mean {mean} is not a scalar')
		stddev = type_support.toScalar(stddev, f'Normal stddev {stddev} is not a scalar')
		super().__init__(mean, stddev, valueType=float)
		self.mean = mean
		self.stddev = stddev

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	if isinstance(self.mean, Samplable) and not isConditioned(self.mean):
	# 		self.mean.conditionforSMT(condition, conditioned_bool)
	# 	if isinstance(self.stddev, Samplable) and not isConditioned(self.stddev):
	# 		self.stddev.conditionforSMT(condition, conditioned_bool)
	# 	return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		"""to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		"""
		if debug:
			print( "Normal")

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)
		
		if self in cached_variables.keys():
			if debug:
				print( "Normal already cached")
			return cached_variables[self]

		var_name = findVariableName(smt_file_path, cached_variables, 'normal', debug=debug)
		return cacheVarName(cached_variables, self, var_name)

	@staticmethod
	def cdf(mean, stddev, x):
		return (1 + math.erf((x - mean) / (sqrt2 * stddev))) / 2

	@staticmethod
	def cdfinv(mean, stddev, x):
		import scipy	# slow import not often needed
		return mean + (sqrt2 * stddev * scipy.special.erfinv(2*x - 1))

	def clone(self):
		return type(self)(self.mean, self.stddev)

	def bucket(self, buckets=None):
		if not isinstance(self.stddev, float):		# TODO relax restriction?
			raise RuntimeError(f'Cannot bucket Normal with non-constant standard deviation')
		if buckets is None:
			buckets = 5
		if isinstance(buckets, int):
			if buckets < 1:
				raise RuntimeError(f'Invalid buckets for Normal.bucket: {buckets}')
			elif buckets == 1:
				endpoints = []
			elif buckets == 2:
				endpoints = [0]
			else:
				left = self.stddev * (-(buckets-3)/2 - 0.5)
				right = self.stddev * ((buckets-3)/2 + 0.5)
				endpoints = numpy.linspace(left, right, buckets-1)
		else:
			endpoints = tuple(buckets)
			for i, v in enumerate(endpoints[:-1]):
				if v >= endpoints[i+1]:
					raise RuntimeError('Non-increasing bucket endpoints for '
					                   f'Normal.bucket: {endpoints}')
		if len(endpoints) == 0:
			return Options([self.clone()])
		buckets = [(-math.inf, endpoints[0])]
		buckets.extend((v, endpoints[i+1]) for i, v in enumerate(endpoints[:-1]))
		buckets.append((endpoints[-1], math.inf))
		pieces = []
		probs = []
		for left, right in buckets:
			pieces.append(self.mean + TruncatedNormal(0, self.stddev, left, right))
			prob = (Normal.cdf(0, self.stddev, right)
			        - Normal.cdf(0, self.stddev, left))
			probs.append(prob)
		assert math.isclose(math.fsum(probs), 1), probs
		return Options(dict(zip(pieces, probs)))

	def sampleGiven(self, value):
		return random.gauss(value[self.mean], value[self.stddev])

	def evaluateInner(self, context):
		mean = valueInContext(self.mean, context)
		stddev = valueInContext(self.stddev, context)
		return Normal(mean, stddev)

	def isEquivalentTo(self, other):
		if not type(other) is Normal:
			return False
		return (areEquivalent(self.mean, other.mean)
			and areEquivalent(self.stddev, other.stddev))

	def __str__(self):
		return f'Normal({self.mean}, {self.stddev})'

class TruncatedNormal(Normal):
	"""Truncated normal distribution."""
	def __init__(self, mean, stddev, low, high):
		if (not isinstance(low, (float, int))
		    or not isinstance(high, (float, int))):	# TODO relax restriction?
			raise RuntimeError('Endpoints of TruncatedNormal must be constant')
		super().__init__(mean, stddev)
		self.low = low
		self.high = high

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	if isinstance(self.low, Samplable) and not isConditioned(self.low):
	# 		self.low.conditionforSMT(condition, conditioned_bool)
	# 	if isinstance(self.high, Samplable) and not isConditioned(self.high):
	# 		self.high.conditionforSMT(condition, conditioned_bool)
	# 	return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		"""to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		"""

		if debug:
			print( "TruncatedNormal")

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)
		
		if self in cached_variables.keys():
			if debug:
				print( "TruncatedNormal already cached")
			return cached_variables[self]

		low = checkAndEncodeSMT(smt_file_path, cached_variables, self.low, debug=debug)
		high = checkAndEncodeSMT(smt_file_path, cached_variables, self.high, debug=debug)
		var_name = findVariableName(smt_file_path, cached_variables, 'truncated_normal', debug=debug)

		lower_bound = smt_lessThanEq(low, var_name)
		upper_bound = smt_lessThanEq(var_name, high)
		smt_encoding = smt_assert("and", lower_bound, upper_bound)
		writeSMTtoFile(smt_file_path, smt_encoding)
		return cacheVarName(cached_variables, self, var_name)

	def clone(self):
		return type(self)(self.mean, self.stddev, self.low, self.high)

	def bucket(self, buckets=None):
		if not isinstance(self.stddev, float):		# TODO relax restriction?
			raise RuntimeError('Cannot bucket TruncatedNormal with '
			                   'non-constant standard deviation')
		if buckets is None:
			buckets = 5
		if isinstance(buckets, int):
			if buckets < 1:
				raise RuntimeError(f'Invalid buckets for TruncatedNormal.bucket: {buckets}')
			endpoints = numpy.linspace(self.low, self.high, buckets+1)
		else:
			endpoints = tuple(buckets)
			if len(endpoints) < 2:
				raise RuntimeError('Too few bucket endpoints for '
				                   f'TruncatedNormal.bucket: {endpoints}')
			if endpoints[0] != self.low or endpoints[-1] != self.high:
				raise RuntimeError(f'TruncatedNormal.bucket endpoints {endpoints} '
				                   'do not match domain')
			for i, v in enumerate(endpoints[:-1]):
				if v >= endpoints[i+1]:
					raise RuntimeError('Non-increasing bucket endpoints for '
					                   f'TruncatedNormal.bucket: {endpoints}')
		pieces, probs = [], []
		for i, left in enumerate(endpoints[:-1]):
			right = endpoints[i+1]
			pieces.append(TruncatedNormal(self.mean, self.stddev, left, right))
			prob = (Normal.cdf(self.mean, self.stddev, right)
			        - Normal.cdf(self.mean, self.stddev, left))
			probs.append(prob)
		return Options(dict(zip(pieces, probs)))

	def sampleGiven(self, value):
		# TODO switch to method less prone to underflow?
		mean, stddev = value[self.mean], value[self.stddev]
		alpha = (self.low - mean) / stddev
		beta = (self.high - mean) / stddev
		alpha_cdf = Normal.cdf(0, 1, alpha)
		beta_cdf = Normal.cdf(0, 1, beta)
		if beta_cdf - alpha_cdf < 1e-15:
			warnings.warn('low precision when sampling TruncatedNormal')
		unif = random.random()
		p = alpha_cdf + unif * (beta_cdf - alpha_cdf)
		return mean + (stddev * Normal.cdfinv(0, 1, p))

	def evaluateInner(self, context):
		mean = valueInContext(self.mean, context)
		stddev = valueInContext(self.stddev, context)
		return TruncatedNormal(mean, stddev, self.low, self.high)

	def isEquivalentTo(self, other):
		if not type(other) is TruncatedNormal:
			return False
		return (areEquivalent(self.mean, other.mean)
			and areEquivalent(self.stddev, other.stddev)
			and self.low == other.low and self.high == other.high)

	def __str__(self):
		return f'TruncatedNormal({self.mean}, {self.stddev}, {self.low}, {self.high})'

class DiscreteRange(Distribution):
	"""Distribution over a range of integers."""
	def __init__(self, low, high, weights=None):
		if not isinstance(low, int):
			raise RuntimeError(f'DiscreteRange endpoint {low} is not a constant integer')
		if not isinstance(high, int):
			raise RuntimeError(f'DiscreteRange endpoint {high} is not a constant integer')
		if not low <= high:
			raise RuntimeError(f'DiscreteRange lower bound {low} is above upper bound {high}')
		if weights is None:
			weights = (1,) * (high - low + 1)
		else:
			weights = tuple(weights)
			assert len(weights) == high - low + 1
		super().__init__(valueType=int)
		self.low = low
		self.high = high
		self.weights = weights
		self.cumulativeWeights = tuple(itertools.accumulate(weights))
		self.options = tuple(range(low, high+1))

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	if isinstance(self.low, Samplable) and not isConditioned(self.low):
	# 		self.low.conditionforSMT(condition, conditioned_bool)
	# 	if isinstance(self.high, Samplable) and not isConditioned(self.high):
	# 		self.high.conditionforSMT(condition, conditioned_bool)
	# 	return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		"""to avoid duplicate variable names, check for variable existence in cached_variables dict:
		   cached_variables : key = obj, value = variable_name / key = 'variables', value = list(cached variables so far)
		"""
		if debug:
			print( "DiscreteRange")

		if isConditioned(self) and not isinstance(self._conditioned, Samplable):
			return cacheVarName(cached_variables, self, self._conditioned)

		if self in cached_variables.keys():
			if debug:
				print( "DiscreteRange is already cached")
			return cached_variables[self]

		low = checkAndEncodeSMT(smt_file_path, cached_variables, self.low, debug=debug)
		high = checkAndEncodeSMT(smt_file_path, cached_variables, self.high, debug=debug)
		var_name = findVariableName(smt_file_path, cached_variables, 'discrete_range', "Int", debug=debug)

		lower_bound = smt_lessThanEq(low, var_name)
		upper_bound = smt_lessThanEq(var_name, high)
		smt_encoding = smt_assert("and", lower_bound, upper_bound)
		writeSMTtoFile(smt_file_path, smt_encoding)
		return cacheVarName(cached_variables, self, var_name)

	def __contains__(self, obj):
		return low <= obj839 and obj <= high

	def clone(self):
		return type(self)(self.low, self.high, self.weights)

	def bucket(self, buckets=None):
		return self.clone()		# already bucketed

	def sampleGiven(self, value):
		return random.choices(self.options, cum_weights=self.cumulativeWeights)[0]

	def isEquivalentTo(self, other):
		if not type(other) is DiscreteRange:
			return False
		return (self.low == other.low and self.high == other.high
		        and self.weights == other.weights)

	def __str__(self):
		return f'DiscreteRange({self.low}, {self.high}, {self.weights})'

class Options(MultiplexerDistribution):
	"""Distribution over a finite list of options.

	Specified by a dict giving probabilities; otherwise uniform over a given iterable.
	"""
	def __init__(self, opts):
		if isinstance(opts, dict):
			options, weights = [], []
			for opt, prob in opts.items():
				if not isinstance(prob, (float, int)):
					raise RuntimeParseError(f'discrete distribution weight {prob}'
					                        ' is not a constant number')
				if prob < 0:
					raise RuntimeParseError(f'discrete distribution weight {prob} is negative')
				if prob == 0:
					continue
				options.append(opt)
				weights.append(prob)
			self.optWeights = dict(zip(options, weights))
		else:
			weights = None
			options = tuple(opts)
			self.optWeights = None
		if len(options) == 0:
			raise RuntimeParseError('tried to make discrete distribution over empty domain!')

		index = self.makeSelector(len(options)-1, weights)
		super().__init__(index, options)

	def checkOptionsType(self, class_type):
		output_bool = True
		for opt in self.options:
			if not isinstance(opt, class_type):
				output_bool = False
				break
		return output_bool

	def encodeToSMT(self, smt_file_path, cached_variables, smt_var=None, debug=False, encode=True):
		if debug:
			print( "Options class")
			if isConditioned(self):
				print( str(self._conditioned))

		if encode and self in cached_variables.keys():
			return cached_variables[self]

		import scenic.domains.driving.roads as roads
		import shapely.geometry

		if encode:
			if self.checkOptionsType(roads.NetworkElement):
				valid_options = []
				regionAroundEgo = cached_variables['regionAroundEgo'].polygon
				import scenic.core.vectors as vectors

				if not isinstance(self._conditioned, vectors.Vector):
					# then find the network elem that intersects with ego's visible
					for reg in self.options:
						intersection = regionAroundEgo.intersection(reg.polygon)
						if not (intersection.is_empty):
							valid_options.append(intersection)

					# import matplotlib.pyplot as plt
					# import shapely.geometry.polygon as polygon
					# if debug:
					# for elem in valid_options:
					# 	if isinstance(elem.polygon, polygon.Polygon):
					# 		plt.plot(*elem.polygon.exterior.xy)
					# 	else:
					# 		for geom in elem.polygon.geoms:
					# 			plt.plot(*geom.exterior.xy)
					# plt.plot(*regionAroundEgo.exterior.xy)
					# plt.show()

					if debug:
						print( "valid_options: "+str(valid_options))

					if len(valid_options) == 0:
						writeSMTtoFile(smt_file_path, "len(valid_options)==0")
						return None

				else:
					vector = self._conditioned
					output_smt = vector.encodeToSMT(smt_file_path, cached_variables, debug)
					return cacheVarName(cached_variables, self, output_smt)

				import scenic.core.regions as regions
				import shapely.geometry

				if smt_var is None:
					x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
					y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
					smt_var = (x,y)

				polygonalRegions = []
				for elem in valid_options:
					if isinstance(elem, shapely.geometry.multipolygon.MultiPolygon):
						for geom in elem.geoms:
							polygonalRegions.append(geom)
					elif isinstance(elem, shapely.geometry.polygon.Polygon):
						polygonalRegions.append(elem)
					else:
						print("elem: ", elem)
						raise NotImplementedError

				if debug:
					print("in Options class, polygonalRegions: ", polygonalRegions)
				multipolygon = shapely.geometry.multipolygon.MultiPolygon(polygonalRegions)
				polygonReg = regions.regionFromShapelyObject(multipolygon)
				polygonReg.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug)

				# polygonalRegions = [regions.regionFromShapelyObject(elem.polygon) for elem in valid_options.options]
				# for polygonReg in polygonalRegions:
				# 	polygonReg.encodeToSMT(smt_file_path, cached_variables, smt_var, debug=debug)
				
				return cacheVarName(cached_variables, self, smt_var)

			elif self.checkOptionsType(class_type = float) or self.checkOptionsType(class_type = int):
				cumulative_smt_encoding = None
				count = 0
				smt_var = findVariableName(smt_file_path, cached_variables, 'options', debug=debug)

				for opt in self.options:
					smt_encoding = smt_equal(smt_var, str(opt))
					if count == 0:
						cumulative_smt_encoding = smt_encoding
					else:
						cumulative_smt_encoding = smt_or(smt_encoding, cumulative_smt_encoding)
					count += 1

				cumulative_smt_encoding = smt_assert(None, cumulative_smt_encoding)
				writeSMTtoFile(smt_file_path, cumulative_smt_encoding)
				return cacheVarName(cached_variables, self, smt_var)

			else: 
				raise NotImplementedError

		else:
			if self.checkOptionsType(roads.NetworkElement):
				valid_options = []
				regionAroundEgo = cached_variables['regionAroundEgo_polygon']
				import scenic.core.vectors as vectors
				if not isinstance(self._conditioned, vectors.Vector):
					if debug:
						print("Options class encode=False, not conditioned on Vector")
					for reg in self.options:
						intersection = regionAroundEgo.intersection(reg.polygon)
						if not (intersection.is_empty):
							valid_options.append(reg)

					# import matplotlib.pyplot as plt
					# import shapely.geometry.polygon as polygon
					# # if debug:
					# for elem in valid_options:
					# 	if isinstance(elem.polygon, polygon.Polygon):
					# 		plt.plot(*elem.polygon.exterior.xy)
					# 	else:
					# 		for geom in elem.polygon.geoms:
					# 			plt.plot(*geom.exterior.xy)

					# plt.plot(*regionAroundEgo.exterior.xy)
					# plt.show()

				else:
					if debug:
						print("Options class encode=False, it IS conditioned on Vector")
					vector = self._conditioned
					for reg in self.options:
						if reg.containsPoint(vector):
							valid_options.append(reg)

				if len(valid_options) == 0:
					return None

				if debug:
					print("valid_options: ", valid_options)

				return Options(valid_options)

			else:
				print("Options NEW CASE: self.options: ", self.options)
				raise NotImplementedError

		return None

	# def conditionforSMT(self, condition, conditioned_bool):
	# 	import scenic.domains.driving.roads as roads
	# 	import scenic.core.vectors as vectors

	# 	if isinstance(condition, vectors.Vector):
	# 		# if a vector is given to condition, then search through all options of regions and 
	# 		# condition to the one that contains 
	# 		import scenic.core.regions as regions
	# 		satisfying_options = []

	# 		for opt in self.options:
	# 			assert isinstance(opt, regions.Region)
	# 			if opt.containsPoint(condition):
	# 				satisfying_options.append(opt)
	# 				conditioned_bool = True
				
	# 			if satisfying_options != []:
	# 				self.conditionTo(satisfying_options)
	# 				return None

	# 	raise NotImplementedError

	@staticmethod
	def makeSelector(n, weights):
		return DiscreteRange(0, n, weights)

	def clone(self):
		return type(self)(self.optWeights if self.optWeights else self.options)

	def bucket(self, buckets=None):
		return self.clone()		# already bucketed

	def evaluateInner(self, context):
		if self.optWeights is None:
			return type(self)(valueInContext(opt, context) for opt in self.options)
		else:
			return type(self)({valueInContext(opt, context): wt
			                  for opt, wt in self.optWeights.items() })

	def isEquivalentTo(self, other):
		if not type(other) == type(self):
			return False
		return (areEquivalent(self.index, other.index)
		        and areEquivalent(self.options, other.options))

	def __str__(self):
		if self.optWeights is not None:
			return f'{type(self).__name__}({self.optWeights})'
		else:
			return f'{type(self).__name__}{argsToString(self.options)}'

@unpacksDistributions
def Uniform(*opts):
	"""Uniform distribution over a finite list of options.

	Implemented as an instance of :obj:`Options` when the set of options is known
	statically, and an instance of `UniformDistribution` otherwise.
	"""
	if any(isinstance(opt, StarredDistribution) for opt in opts):
		return UniformDistribution(opts)
	else:
		return Options(opts)

class UniformDistribution(Distribution):
	"""Uniform distribution over a variable number of options.

	See :obj:`Options` for the more common uniform distribution over a fixed number
	of options. This class is for the special case where iterable unpacking is
	applied to a distribution, so that the number of options is unknown at
	compile time.
	"""
	def __init__(self, opts):
		self.options = opts
		valueType = type_support.unifyingType(self.options)
		super().__init__(*self.options, valueType=valueType)

	def checkOptionsType(self, class_type):
		output_bool = True
		for opt in self.options:
			sample = opt.sample()
			if isinstance(sample,tuple) and not isinstance(sample, class_type):
				output_bool = False
				break
			if not isinstance(opt.sample(), class_type):
				output_bool = False
				break
		return output_bool

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False, encode = True):
		if debug:
			print( "class UniformDistribution encodeToSMT")

		if self in set(cached_variables.keys()):
			if debug:
				print( "UniformDistribution already cached")
			return cached_variables[self]

		# Assume that only one StarredDist is an element of self.options
		if len(self.options) != 1:
			raise NotImplementedError

		output_smt = None
		if not isConditioned(self):
			output_smt = self.options[0].encodeToSMT(smt_file_path, cached_variables, debug, encode)
		else:
			output_smt = self._conditioned.encodeToSMT(smt_file_path, cached_variables, debug, encode)

		if output_smt is None:
			return None
		
		return cacheVarName(cached_variables, self, output_smt)

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def sampleGiven(self, value):
		opts = []
		for opt in self.options:
			if isinstance(opt, StarredDistribution):
				opts.extend(value[opt])
			else:
				opts.append(value[opt])
		if not opts:
			raise RejectionException('uniform distribution over empty domain')
		return random.choice(opts)

	def evaluateInner(self, context):
		opts = tuple(valueInContext(opt, context) for opt in self.options)
		return UniformDistribution(opts)

	def __str__(self):
		return f'UniformDistribution({self.options})'
