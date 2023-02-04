"""Scenic vectors and vector fields."""

from __future__ import annotations

import math
from math import sin, cos
import random
import collections
import itertools

import shapely.geometry
import wrapt

from scenic.core.distributions import (Samplable, Distribution, MethodDistribution,
    needsSampling, makeOperatorHandler, distributionMethod, distributionFunction,
	RejectionException, smt_add, smt_subtract, smt_multiply, smt_divide, smt_and, 
	smt_equal, smt_assert, findVariableName, isConditioned,
	checkAndEncodeSMT, writeSMTtoFile, cacheVarName, smt_lessThan, smt_lessThanEq, smt_ite, normalizeAngle_SMT, vector_operation_smt)
from scenic.core.lazy_eval import valueInContext, needsLazyEvaluation, makeDelayedFunctionCall
import scenic.core.utils as utils
from scenic.core.geometry import normalizeAngle
from scenic.core.utils import areEquivalent

class VectorDistribution(Distribution):
	"""A distribution over Vectors."""
	defaultValueType = None		# will be set after Vector is defined

	def toVector(self):
		return self

class CustomVectorDistribution(VectorDistribution):
	"""Distribution with a custom sampler given by an arbitrary function."""
	def __init__(self, sampler, *dependencies, name='CustomVectorDistribution', evaluator=None):
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

	def __str__(self):
		deps = utils.argsToString(self.dependencies)
		return f'{self.name}{deps}'

class VectorOperatorDistribution(VectorDistribution):
	"""Vector version of OperatorDistribution."""
	def __init__(self, operator, obj, operands):
		super().__init__(obj, *operands)
		self.operator = operator
		self.object = obj
		self.operands = operands

	def conditionforSMT(self, condition, conditioned_bool):
		if isinstance(self.object, Samplable) and not isConditioned(self.object):
			self.object.conditionforSMT(condition, conditioned_bool)
		for op in self.operands:
			if isinstance(op, Samplable) and not isConditioned(op):
				op.conditionforSMT(condition, conditioned_bool)
		return None

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False, encode=True):
		# if not isinstance(obj, Samplable):
		# 	obj = self

		if debug:
			print( "VectorOperatorDistribution")
			print( "self.object: "+str(self.object))
			print( "self.operator: "+str(self.operator))
			print( "self.operands: "+str(self.operands))

		if isConditioned(self) and isinstance(self._conditioned, Vector):
			if debug:
				print("self is conditioned to vector: ", self._conditioned)
			vector = self._conditioned
			vector_smt = (str(vector.x), str(vector.y))
			return cacheVarName(cached_variables, self, vector_smt)

		if not encode:
			return self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug, encode=encode)

		if self in cached_variables.keys():
			if debug:
				print( "Already In cached_variables")
			return cached_variables[self]

		obj = self.object.encodeToSMT(smt_file_path, cached_variables, debug=debug)
		if obj is None:
			return None

		if debug:
			print("VectorOperatorDistribution obj: ", obj)
		obj = Vector(obj[0], obj[1])

		### Make sure object and operands are smt encoded
		if self.operator == 'rotatedBy':
			angle = self.operands[0]
			# it is expected that the self.object in this case is of Vector class
			output_vector = obj.rotatedByEncodeToSMT(cached_variables, smt_file_path, angle, debug=debug)
			if debug:
				print( "rotatedBy")
			return cacheVarName(cached_variables, self, output_vector)

		elif self.operator == 'offsetRotated':
			if debug:
				print( "offsetRotated")
			heading = self.operands[0]
			offset = self.operands[1]
			output_vector = obj.offsetRotatedEncodeToSMT(cached_variables, smt_file_path, heading, offset, debug=debug)
			return cacheVarName(cached_variables, self, output_vector)

		elif self.operator == 'offsetRadially':
			if debug:
				print( "offsetRadially")
			radius = self.operands[0]
			heading = self.operands[1]
			output_vector = obj.offsetRadiallyEncodeSMT(cached_variables, smt_file_path, radius, heading, debug=debug)
			return cacheVarName(cached_variables, self, output_vector)

		elif self.operator == 'distanceTo':
			if debug:
				print( "distanceTo")

			assert(isinstance(self.operands[0], Vector))
			output = obj.distanceToEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == 'angleWith':
			if debug:
				print( "angleWith")

			assert(isinstance(self.operands[0], Vector))
			output_angle = obj.angleWithEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output_angle)

		elif self.operator == 'angleTo':
			if debug:
				print( "angleTo")
			assert(isinstance(self.operands[0], Vector))
			output_angle = obj.angleToEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output_angle)

		elif self.operator == 'norm':
			if debug:
				print( "norm")
			output = obj.normEncodeSMT(smt_file_path, cached_variables, debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == 'normalized':
			if debug:
				print( "normalized")
			output = obj.normalizedEncodeSMT(smt_file_path, cached_variables, debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__add__' or '__radd__':
			if debug:
				print( "add or radd")
			output = obj.addEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__sub__':
			if debug:
				print( "subtract")

			output = obj.subEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__rsub__':
			if debug:
				print( "rsubtract")
			output = obj.rsubEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__mul__' or '__rmul__':
			if debug:
				print( "multiply or rmultiply")
			output = obj.mulEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__truediv__':
			if debug:
				print( "__truediv__")
			output = obj.truedivEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__len__':
			if debug:
				print( "__len__")
			output = obj.lenEncodeSMT(smt_file_path, cached_variables, debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__getitem__':
			if debug:
				print( "__getitem__")
			output = obj.subEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		elif self.operator == '__eq__':
			if debug:
				print( "__eq__")
			output = obj.subEncodeSMT(smt_file_path, cached_variables, self.operands[0], debug=debug)
			return cacheVarName(cached_variables, self, output)

		else: 
			raise NotImplementedError

		return None

	def sampleGiven(self, value):
		first = value[self.object]
		rest = (value[child] for child in self.operands)
		op = getattr(first, self.operator)
		return op(*rest)

	def evaluateInner(self, context):
		obj = valueInContext(self.object, context)
		operands = tuple(valueInContext(arg, context) for arg in self.operands)
		return VectorOperatorDistribution(self.operator, obj, operands)

	def isEquivalentTo(self, other):
		if not type(other) is VectorOperatorDistribution:
			return False
		return (areEquivalent(self.object, other.object)
			and areEquivalent(self.operator, other.operator)
			and areEquivalent(self.operands, other.operands))

	def __str__(self):
		ops = utils.argsToString(self.operands)
		return f'{self.object}.{self.operator}{ops}'

class VectorMethodDistribution(VectorDistribution):
	"""Vector version of MethodDistribution."""
	def __init__(self, method, obj, args, kwargs):
		super().__init__(*args, *kwargs.values())
		self.method = method
		self.object = obj
		self.arguments = args
		self.kwargs = kwargs

	def conditionforSMT(self, condition, conditioned_bool):
		if isinstance(self.object, Samplable) and not isConditioned(self.object):
			self.object.conditionforSMT(condition, conditioned_bool)
		for arg in self.arguments:
			if isinstance(arg, Samplable) and not isConditioned(arg):
				arg.conditionforSMT(condition, conditioned_bool)
		for kwarg in self.kwargs:
			if isinstance(kwarg, Samplable) and not isConditioned(kwarg):
				kwarg.conditionforSMT(condition, conditioned_bool)
		return None

	def isEquivalentTo(self, other):
		if not type(other) is VectorMethodDistribution:
			return False
		return (areEquivalent(self.object, other.object)
			and areEquivalent(self.method, other.method)
			and areEquivalent(self.kwargs, other.kwargs))

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "VectorMethodDistribution encodeToSMT")
			print( "self.method: "+str(self.method))
			print( "self.method type: "+str(type(self.method)))
			print( "self.object: "+str(self.object))
			for arg in self.arguments:
				print( "self.argument: "+str(arg))
			for kwarg in self.kwargs:
				print( "self.kwarg: "+str(kwarg))

		if self.method == Vector.offsetRotated:
			output = self.object.offsetRotatedEncodeToSMT(cached_variables, smt_file_path, *self.arguments)
			return cacheVarName(cached_variables, self, output)
		else:
			raise NotImplementedError

		return None

	def sampleGiven(self, value):
		args = (value[arg] for arg in self.arguments)
		kwargs = { name: value[arg] for name, arg in self.kwargs.items() }
		return self.method(self.object, *args, **kwargs)

	def evaluateInner(self, context):
		obj = valueInContext(self.object, context)
		arguments = tuple(valueInContext(arg, context) for arg in self.arguments)
		kwargs = { name: valueInContext(arg, context) for name, arg in self.kwargs.items() }
		return VectorMethodDistribution(self.method, obj, arguments, kwargs)

	def __str__(self):
		args = utils.argsToString(itertools.chain(self.arguments, self.kwargs.values()))
		return f'{self.object}.{self.method.__name__}{args}'

def scalarOperator(method):
	"""Decorator for vector operators that yield scalars."""
	op = method.__name__
	setattr(VectorDistribution, op, makeOperatorHandler(op))

	@wrapt.decorator
	def wrapper(wrapped, instance, args, kwargs):
		if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
			return MethodDistribution(method, instance, args, kwargs)
		else:
			return wrapped(*args, **kwargs)
	return wrapper(method)

def makeVectorOperatorHandler(op):
	def handler(self, *args):
		return VectorOperatorDistribution(op, self, args)
	return handler
def vectorOperator(method):
	"""Decorator for vector operators that yield vectors."""
	op = method.__name__
	setattr(VectorDistribution, op, makeVectorOperatorHandler(op))

	@wrapt.decorator
	def wrapper(wrapped, instance, args, kwargs):
		def helper(*args):
			if needsSampling(instance):
				return VectorOperatorDistribution(op, instance, args)
			elif any(needsSampling(arg) for arg in args):
				return VectorMethodDistribution(method, instance, args, {})
			elif any(needsLazyEvaluation(arg) for arg in args):
				# see analogous comment in distributionFunction
				return makeDelayedFunctionCall(helper, args, {})
			else:
				return wrapped(*args)
		return helper(*args)
	return wrapper(method)

def vectorDistributionMethod(method):
	"""Decorator for methods that produce vectors. See distributionMethod."""
	@wrapt.decorator
	def wrapper(wrapped, instance, args, kwargs):
		def helper(*args, **kwargs):
			if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
				return VectorMethodDistribution(method, instance, args, kwargs)
			elif any(needsLazyEvaluation(arg)
			         for arg in itertools.chain(args, kwargs.values())):
				# see analogous comment in distributionFunction
				return makeDelayedFunctionCall(helper, args, kwargs)
			else:
				return wrapped(*args, **kwargs)
		return helper(*args, **kwargs)
	return wrapper(method)

class Vector(Samplable, collections.abc.Sequence):
	"""A 2D vector, whose coordinates can be distributions."""
	def __init__(self, x, y):
		self.coordinates = (x, y)
		super().__init__(self.coordinates)

	@property
	def x(self) -> float:
		return self.coordinates[0]

	@property
	def y(self) -> float:
		return self.coordinates[1]

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "Vector Class")

		if self in set(cached_variables.keys()):
			if debug:
				print( "Vector already cached")
			return cached_variables[self]

		x = checkAndEncodeSMT(smt_file_path, cached_variables, self.x, debug = debug)
		y = checkAndEncodeSMT(smt_file_path, cached_variables, self.y, debug = debug)
		return cacheVarName(cached_variables, self, (x,y))


	def toVector(self) -> Vector:
		return self

	def sampleGiven(self, value):
		return Vector(*(value[coord] for coord in self.coordinates))

	def evaluateInner(self, context):
		return Vector(*(valueInContext(coord, context) for coord in self.coordinates))

	@vectorOperator
	def rotatedBy(self, angle) -> Vector:
		"""Return a vector equal to this one rotated counterclockwise by the given angle."""
		x, y = self.x, self.y
		c, s = cos(angle), sin(angle)
		return Vector((c * x) - (s * y), (s * x) + (c * y))

	def rotatedByEncodeToSMT(self, cached_variables, smt_file_path, angle_obj, debug=False):
		""" encodes rotatedBy function to a SMT formula 
		type: angle:= class objects """
		if debug:
			print( "rotatedByEncodeToSMT()")

		angle = checkAndEncodeSMT(smt_file_path, cached_variables, angle_obj, debug = debug)
		x = checkAndEncodeSMT(smt_file_path, cached_variables, self.x, debug = debug)
		y = checkAndEncodeSMT(smt_file_path, cached_variables, self.y, debug = debug)

		if debug:
			print("angle: ", angle)
			print("x: ", x)
			print("y: ", y)

		cos = "(cos "+angle+")"
		sin = "(sin "+angle+")"

		cos_mul_x = smt_multiply(cos, x)
		sin_mul_y = smt_multiply(sin, y)
		cos_mul_y = smt_multiply(cos, y)
		sin_mul_x = smt_multiply(sin, x)

		x_name = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		y_name = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		x_smt_encoding = smt_assert("equal", x_name, smt_subtract(cos_mul_x, sin_mul_y))
		y_smt_encoding = smt_assert("equal", y_name, smt_add(sin_mul_x, cos_mul_y))

		writeSMTtoFile(smt_file_path, x_smt_encoding)
		writeSMTtoFile(smt_file_path, y_smt_encoding)

		return (x_name, y_name)

	@vectorOperator
	def offsetRotated(self, heading, offset) -> Vector:
		ro = offset.rotatedBy(heading)
		return self + ro

	def offsetRotatedEncodeToSMT(self, cached_variables, smt_file_path, heading, offset, debug=False):
		""" type(heading), type(offset) : objects
		"""
		if debug:
			print( "offsetRotatedEncodeToSMT()")

		rotated_offset_smt = offset.rotatedByEncodeToSMT(cached_variables, smt_file_path, heading, debug = debug)
		x_smt_var = rotated_offset_smt[0]
		y_smt_var = rotated_offset_smt[1]

		self_x = checkAndEncodeSMT(smt_file_path, cached_variables, self.x, debug = debug)
		self_y = checkAndEncodeSMT(smt_file_path, cached_variables, self.y, debug = debug)

		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		x_smt_encoding = smt_assert("equal", output_x, smt_add(x_smt_var, self_x))
		y_smt_encoding = smt_assert("equal", output_y, smt_add(y_smt_var, self_y))

		writeSMTtoFile(smt_file_path, x_smt_encoding)
		writeSMTtoFile(smt_file_path, y_smt_encoding)
		return (output_x, output_y)

	@vectorOperator
	def offsetRadially(self, radius, heading) -> Vector:
		return self.offsetRotated(heading, Vector(0, radius))

	def offsetRadiallyEncodeSMT(self, cached_variables, smt_file_path, radius, heading, debug=False):
		offset = Vector(0, radius)
		return self.offsetRotatedEncodeToSMT(cached_variables, smt_file_path, heading, offset, debug=False)

	@scalarOperator
	def distanceTo(self, other) -> float:
		if not isinstance(other, Vector):
			return other.distanceTo(self)
		dx, dy = other.toVector() - self
		return math.hypot(dx, dy)

	def distanceToEncodeSMT(smt_file_path, cached_variables, vector, debug=False):
		""" type(vector) = Vector Class """

		assert(isinstance(vector, Vector))

		# distance * distance = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
		(other_x, other_y) = vector.encodeToSMT(smt_file_path, cached_variables, debug=debug)
		(x, y) = self.encodeToSMT(smt_file_path, cached_variables, debug=debug)

		x1_x2 = smt_subtract(x, other_x)
		sq_x1_x2 = smt_multiply(x1_x2, x1_x2)
		y1_y2 = smt_subtract(y, other_y)
		sq_y1_y2 = smt_multiply(y1_y2, y1_y2)
		summation = smt_add(sq_x1_x2, sq_y1_y2)

		output_dist = findVariableName(smt_file_path, cached_variables, 'distance', debug=debug)
		sq_var_name = smt_multiply(output_dist, output_dist)
		smt_encoding = smt_assert("equal", sq_var_name, summation)
		writeSMTtoFile(smt_file_path, smt_encoding)
		return output_dist

	@scalarOperator
	def angleTo(self, other) -> float:
		dx, dy = other.toVector() - self
		return normalizeAngle(math.atan2(dy, dx) - (math.pi / 2))

	def angleToEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "angleTo")          
		# (other_x, other_y) = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		# (vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		# dx = smt_subtract(other_x, vec_x)
		# dy = smt_subtract(other_y, vec_y)
		# smt_atan = "(arctan "+dy+" "+dx+")"
		# subtraction = smt_subtract(smt_atan, smt_divide('3.1416','2'))
		raise NotImplementedError
		# theta = normalizeAngle_SMT(smt_file_path, cached_variables, subtraction, debug=debug)
		theta = findVariableName(smt_file_path, cached_variables, 'theta', debug=debug)
		writeSMTtoFile(smt_file_path, smt_assert("equal", theta, subtraction))
		return theta

	@scalarOperator
	def angleWith(self, other) -> float:
		"""Compute the signed angle between self and other.

		The angle is positive if other is counterclockwise of self (considering
		the smallest possible rotation to align them).
		"""
		x, y = self.x, self.y
		ox, oy = other.x, other.y
		return normalizeAngle(math.atan2(oy, ox) - math.atan2(y, x))

	def angleWithEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "angleWith")

		(other_x, other_y) = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		(vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		smt_atan_other = "(arctan (div "+smt_divide(other_y, other_x)+")" 
		smt_atan_vec   = "(arctan (div "+smt_divide(vec_y, vec_x)+")" 
		subtraction = smt_subtract(smt_atan_other, smt_atan_vec)
		theta = normalizeAngle_SMT(smt_file_path, cached_variables, subtraction, debug=debug)
		return theta

	@scalarOperator
	def norm(self) -> float:
		return math.hypot(*self.coordinates)

	def normEncodeSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "normEncodeSMT")
		(vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		square_x = smt_multiply(vec_x, vec_x)
		square_y = smt_multiply(vec_y, vec_y)
		summation = smt_add(square_x, square_y)
		norm_var = findVariableName(smt_file_path, cached_variables, 'vec_norm', debug=debug)
		sq_norm_var = smt_multiply(norm_var, norm_var)
		smt_encoding = smt_assert("equal", sq_norm_var, summation)
		writeSMTtoFile(smt_file_path, smt_encoding)
		return norm_var

	@vectorOperator
	def normalized(self) -> Vector:
		l = math.hypot(*self.coordinates)
		return Vector(*(coord/l for coord in self.coordinates))

	def normalizedEncodeSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "normalizedEncodeSMT")
		(vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		square_x = smt_multiply(vec_x, vec_x)
		square_y = smt_multiply(vec_y, vec_y) 
		summation = smt_add(square_x, square_y)
		norm_var = findVariableName(smt_file_path, cached_variables, 'vec_norm', debug=debug)
		sq_norm_var = smt_multiply(norm_var, norm_var) 
		norm_smt_encoding = smt_assert("equal", sq_norm_var, summation)
		x = smt_divide(vec_x, norm_var)
		y = smt_divide(vec_y, norm_var)
		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)
		smt_x = smt_assert("equal", output_x, x)
		smt_y = smt_assert("equal", output_y, y)
		writeSMTtoFile(smt_file_path, norm_smt_encoding)
		writeSMTtoFile(smt_file_path, smt_x)
		writeSMTtoFile(smt_file_path, smt_y)
		return (output_x, output_y)

	@vectorOperator
	def __add__(self, other) -> Vector:
		return Vector(self[0] + other[0], self[1] + other[1])

	def addEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "addEncodeSMT")

		# variable can be a constant or Vector
		variable = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		self_vector = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		summation_vector = vector_operation_smt(self_vector, "add", variable)

		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		(x, y) = vector_operation_smt( (output_x, output_y), "equal", summation_vector)
		writeSMTtoFile(smt_file_path, smt_assert(None, x))
		writeSMTtoFile(smt_file_path, smt_assert(None, y))
		return (output_x, output_y)


	@vectorOperator
	def __radd__(self, other) -> Vector:
		return Vector(self[0] + other[0], self[1] + other[1])

	def raddEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "raddEncodeSMT")
		return self.addEncodeSMT(self, smt_file_path, cached_variables, other, debug=debug)

	@vectorOperator
	def __sub__(self, other) -> Vector:
		return Vector(self[0] - other[0], self[1] - other[1])

	def subEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "subEncodeSMT")
		# variable can be a constant or Vector
		variable = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		self_vector = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		subtraction_vector = vector_operation_smt(self_vector, "subtract", variable)

		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		(x, y) = vector_operation_smt( (output_x, output_y), "equal", subtraction_vector)
		writeSMTtoFile(smt_file_path, smt_assert(None, x))
		writeSMTtoFile(smt_file_path, smt_assert(None, y))
		return (output_x, output_y)

	@vectorOperator
	def __rsub__(self, other) -> Vector:
		return Vector(other[0] - self[0], other[1] - self[1])

	def rsubEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "rsubEncodeSMT")
		# variable can be a constant or Vector
		variable = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		self_vector = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		subtraction_vector = vector_operation_smt(variable, "subtract", self_vector)

		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		(x, y) = vector_operation_smt( (output_x, output_y), "equal", subtraction_vector)
		writeSMTtoFile(smt_file_path, smt_assert(None, x))
		writeSMTtoFile(smt_file_path, smt_assert(None, y))
		return (output_x, output_y)

	@vectorOperator
	def __mul__(self, other) -> Vector:
		return Vector(*(coord*other for coord in self.coordinates))

	def mulEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "mulEncodeSMT")
		scalar = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		(vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		mul_x = smt_multiply(scalar, vec_x)
		mul_y = smt_multiply(scalar, vec_y)
		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		x_smt_encoding = smt_assert("equal", output_x, mul_x)
		y_smt_encoding = smt_assert("equal", output_y, mul_y)
		writeSMTtoFile(smt_file_path, x_smt_encoding)
		writeSMTtoFile(smt_file_path, y_smt_encoding)
		return (output_x, output_y)

	def __rmul__(self, other) -> Vector:
		return self.__mul__(other)

	def rmulEncodeSMT(self, smt_file_path, cached_variables, other,debug=False):
		if debug:
			print( "rmulEncodeSMT")
		return self.mulEncodeSMT(smt_file_path, cached_variables, other, debug=False)

	@vectorOperator
	def __truediv__(self, other) -> Vector:
		return Vector(*(coord/other for coord in self.coordinates))

	def truedivEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "truedivEncodeSMT")
		scalar = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		(vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		div_x = smt_divide(vec_x, scalar)
		div_y = smt_divide(vec_y, scalar)
		output_x = findVariableName(smt_file_path, cached_variables, 'x', debug=debug)
		output_y = findVariableName(smt_file_path, cached_variables, 'y', debug=debug)

		x_smt_encoding = smt_assert("equal", output_x, div_x)
		y_smt_encoding = smt_assert("equal", output_y, div_y)
		writeSMTtoFile(smt_file_path, x_smt_encoding)
		writeSMTtoFile(smt_file_path, y_smt_encoding)
		return (output_x, output_y)

	def __len__(self):
		return len(self.coordinates)

	def lenEncodeSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "lenEncodeSMT")
		length_var = findVariableName(smt_file_path, cached_variables, 'vec_length', debug=debug)
		smt_encoding = smt_assert("equal", length_var, str(len(self.coordinates)))
		writeSMTtoFile(smt_file_path, smt_encoding)
		return length_var

	def __getitem__(self, index):
		return self.coordinates[index]

	def getitemEncodeSMT(self, smt_file_path, cached_variables, index, debug=False):
		if debug:
			print( "getitemEncodeSMT")
		index = checkAndEncodeSMT(smt_file_path, cached_variables, index, debug = debug)
		vec = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		element = findVariableName(smt_file_path, cached_variables, 'vec_element', debug=debug)

		## TODO: Encode Array!
		raise NotImplementedError
		smt_encoding = smt_assert("equal", element, vec[index])
		writeSMTtoFile(smt_file_path, smt_encoding)
		return cacheVarName(cached_variables, self, (element))

	def __repr__(self):
		return f'({self.x} @ {self.y})'

	def __eq__(self, other):
		if isinstance(other, Vector):
			return other.coordinates == self.coordinates
		elif isinstance(other, (tuple, list)):
			return tuple(other) == self.coordinates
		else:
			return NotImplemented

	def eqEncodeSMT(self, smt_file_path, cached_variables, other, debug=False):
		if debug:
			print( "equal")
		(vec_x, vec_y) = checkAndEncodeSMT(smt_file_path, cached_variables, self, debug = debug)
		if isinstance(other, Vector):
			(other_x, other_y) = checkAndEncodeSMT(smt_file_path, cached_variables, other, debug = debug)
		elif isinstance(other, (list, tuple)):
			if len(other) == 2: 
				other_x = checkAndEncodeSMT(smt_file_path, cached_variables, other[0], debug = debug)
				other_y = checkAndEncodeSMT(smt_file_path, cached_variables, other[1], debug = debug)
			else:
				raise NotImplementedError
		else:
			raise NotImplementedError

		x_eq = smt_equal(vec_x, other_x)
		y_eq = smt_equal(vec_y, other_y)
		eq_smt = smt_and(x_eq, y_eq)
		eq_var = findVariableName(smt_file_path, cached_variables, "eq_bool", class_type = "Bool", debug=debug)
		smt_encoding = smt_assert("equal", eq_var, eq_smt)

		writeSMTtoFile(smt_file_path, smt_encoding)
		return eq_var

	def __hash__(self):
		return hash(self.coordinates)

VectorDistribution.defaultValueType = Vector

class OrientedVector(Vector):
	def __init__(self, x, y, heading):
		super().__init__(x, y)
		self.heading = heading

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "OrientedVector")

		x = checkAndEncodeSMT(smt_file_path, cached_variables, self.x, debug)
		y = checkAndEncodeSMT(smt_file_path, cached_variables, self.y, debug)
		heading = checkAndEncodeSMT(smt_file_path, cached_variables, self.heading, debug)
		return cacheVarName(cached_variables, self, (x, y, heading))

	@staticmethod
	@distributionFunction
	def make(position, heading) -> OrientedVector:
		return OrientedVector(*position, heading)

	def makeEncodeSMT(self, smt_file_path, cached_variables, position, heading, debug=False):
		""" type(position): Vector, type(heading): obj or float or int"""
		if debug:
			print( "OrientedVector makeEncodeSMT")
		pos = checkAndEncodeSMT(smt_file_path, cached_variables, position, debug)
		assert(isinstance(pos, tuple))
		x = pos[0]
		y = pos[1]
		heading = checkAndEncodeSMT(smt_file_path, cached_variables, heading, debug)
		return (x, y, heading)

	def toHeading(self):
		return self.heading

	def toHeadingEncodeSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "Class OrientedVector toHeadingEncodeSMT")

		(x,y,heading) = self.encodeToSMT(smt_file_path, cached_variables, debug=debug)
		return heading

	def __eq__(self, other):
		if type(other) is not OrientedVector:
			return NotImplemented
		return (other.coordinates == self.coordinates
		    and other.heading == self.heading)

	def __hash__(self):
		return hash((self.coordinates, self.heading))

class VectorField:
	"""A vector field, providing a heading at every point.

	Arguments:
		name (str): name for debugging.
		value: function computing the heading at the given `Vector`.
		minSteps (int): Minimum number of steps for `followFrom`; default 4.
		defaultStepSize (float): Default step size for `followFrom`; default 5.
	"""
	def __init__(self, name, value, minSteps=4, defaultStepSize=5):
		self.name = name
		self.value = value
		self.valueType = float
		self.minSteps = minSteps
		self.defaultStepSize = defaultStepSize

	def conditionforSMT(self, condition, conditioned_bool):
		raise NotImplementedError

	def encodeToSMT(self, smt_file_path, cached_variables, debug=False):
		if debug:
			print( "VectorField")

		if self in set(cached_variables.keys()):
			return cached_variables[self]

		var_name = findVariableName(smt_file_path, cached_variables, 'vectorField', debug=debug)

		raise NotImplementedError

		return var_name

	@distributionMethod
	def __getitem__(self, pos) -> float:
		return self.value(pos)

	@vectorDistributionMethod
	def followFrom(self, pos, dist, steps=None, stepSize=None):
		"""Follow the field from a point for a given distance.

		Uses the forward Euler approximation, covering the given distance with
		equal-size steps. The number of steps can be given manually, or computed
		automatically from a desired step size.

		Arguments:
			pos (`Vector`): point to start from.
			dist (float): distance to travel.
			steps (int): number of steps to take, or :obj:`None` to compute the number of
				steps based on the distance (default :obj:`None`).
			stepSize (float): length used to compute how many steps to take, or
				:obj:`None` to use the field's default step size.
		"""
		if steps is None:
			steps = self.minSteps
			stepSize = self.defaultStepSize if stepSize is None else stepSize
			if stepSize is not None:
				steps = max(steps, math.ceil(dist / stepSize))

		step = dist / steps
		for i in range(steps):
			pos = pos.offsetRadially(step, self[pos])
		return pos

	def followFrom_encodeToSMT(self, smt_file_path, cached_variables, pos_obj, dist, debug=False, encode=True):
		assert(isConditioned(pos) and isinstance(pos._conditioned, Vector))
		x, y = pos_obj._conditioned
		pos = Vector(float(x), float(y))
		network = cached_variables['network']

		if steps is None:
			steps = self.minSteps
			stepSize = self.defaultStepSize if stepSize is None else stepSize
			if stepSize is not None:
				steps = max(steps, math.ceil(dist / stepSize))

		step = dist / steps
		for i in range(steps):
			pos = pos.offsetRadially(step, network.nominalDirectionAt(pos))

		return (str(pos.x), str(pos.y))


	@staticmethod
	def forUnionOf(regions):
		"""Creates a `PiecewiseVectorField` from the union of the given regions.

		If none of the regions have an orientation, returns :obj:`None` instead.
		"""
		if any(reg.orientation for reg in regions):
			return PiecewiseVectorField('Union', regions)
		else:
			return None

	def __str__(self):
		return f'<{type(self).__name__} {self.name}>'

class PolygonalVectorField(VectorField):
	"""A piecewise-constant vector field defined over polygonal cells.

	Arguments:
		name (str): name for debugging.
		cells: a sequence of cells, with each cell being a pair consisting of a Shapely
			geometry and a heading. If the heading is :obj:`None`, we call the given
			**headingFunction** for points in the cell instead.
		headingFunction: function computing the heading for points in cells without
			specified headings, if any (default :obj:`None`).
		defaultHeading: heading for points not contained in any cell (default
			:obj:`None`, meaning reject such points).
	"""
	def __init__(self, name, cells, headingFunction=None, defaultHeading=None):
		self.cells = tuple(cells)
		if headingFunction is None and defaultHeading is not None:
			headingFunction = lambda pos: defaultHeading
		self.headingFunction = headingFunction
		for cell, heading in self.cells:
			if heading is None and headingFunction is None and defaultHeading is None:
				raise RuntimeError(f'missing heading for cell of PolygonalVectorField')
		self.defaultHeading = defaultHeading
		super().__init__(name, self.valueAt)

	def valueAt(self, pos):
		point = shapely.geometry.Point(pos)
		for cell, heading in self.cells:
			if cell.intersects(point):
				return self.headingFunction(pos) if heading is None else heading
		if self.defaultHeading is not None:
			return self.defaultHeading
		raise RejectionException(f'evaluated PolygonalVectorField at undefined point')

class PiecewiseVectorField(VectorField):
	"""A vector field defined by patching together several regions.

	The heading at a point is determined by checking each region in turn to see if it has
	an orientation and contains the point, returning the corresponding heading if so. If
	we get through all the regions, then we return the **defaultHeading**, if any, and
	otherwise reject the scene.

	Arguments:
		name (str): name for debugging.
		regions (sequence of `Region` objects): the regions making up the field.
		defaultHeading (float): the heading for points not in any region with an
			orientation (default :obj:`None`, meaning reject such points).
	"""
	def __init__(self, name, regions, defaultHeading=None):
		self.regions = tuple(regions)
		self.defaultHeading = defaultHeading
		super().__init__(name, self.valueAt)

	def valueAt(self, point):
		for region in self.regions:
			if region.containsPoint(point) and region.orientation:
				return region.orientation[point]
		if self.defaultHeading is not None:
			return self.defaultHeading
		raise RejectionException(f'evaluated PiecewiseVectorField at undefined point')
