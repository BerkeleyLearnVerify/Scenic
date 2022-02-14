"""Scenic vectors and vector fields."""

from __future__ import annotations

import math
from math import sin, cos
import random
import collections
import itertools

import decorator
import shapely.geometry

from scenic.core.distributions import (Samplable, Distribution, MethodDistribution,
    needsSampling, makeOperatorHandler, distributionMethod, distributionFunction,
	RejectionException)
from scenic.core.lazy_eval import valueInContext, needsLazyEvaluation, makeDelayedFunctionCall
import scenic.core.utils as utils
from scenic.core.geometry import normalizeAngle

class VectorDistribution(Distribution):
	"""A distribution over Vectors."""
	_defaultValueType = None		# will be set after Vector is defined

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

	def sampleGiven(self, value):
		first = value[self.object]
		rest = (value[child] for child in self.operands)
		op = getattr(first, self.operator)
		return op(*rest)

	def evaluateInner(self, context):
		obj = valueInContext(self.object, context)
		operands = tuple(valueInContext(arg, context) for arg in self.operands)
		return VectorOperatorDistribution(self.operator, obj, operands)

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

	@decorator.decorator
	def wrapper(wrapped, instance, *args, **kwargs):
		if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
			return MethodDistribution(method, instance, args, kwargs)
		else:
			return wrapped(instance, *args, **kwargs)
	return wrapper(method)

def makeVectorOperatorHandler(op):
	def handler(self, *args):
		return VectorOperatorDistribution(op, self, args)
	return handler
def vectorOperator(method):
	"""Decorator for vector operators that yield vectors."""
	op = method.__name__
	setattr(VectorDistribution, op, makeVectorOperatorHandler(op))

	@decorator.decorator
	def wrapper(wrapped, instance, *args, **kwargs):
		def helper(*args):
			if needsSampling(instance):
				return VectorOperatorDistribution(op, instance, args)
			elif any(needsSampling(arg) for arg in args):
				return VectorMethodDistribution(method, instance, args, {})
			elif any(needsLazyEvaluation(arg) for arg in args):
				# see analogous comment in distributionFunction
				return makeDelayedFunctionCall(helper, args, {})
			else:
				return wrapped(instance, *args)
		return helper(*args)
	return wrapper(method)

def vectorDistributionMethod(method):
	"""Decorator for methods that produce vectors. See distributionMethod."""
	@decorator.decorator
	def wrapper(wrapped, instance, *args, **kwargs):
		def helper(*args, **kwargs):
			if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
				return VectorMethodDistribution(method, instance, args, kwargs)
			elif any(needsLazyEvaluation(arg)
			         for arg in itertools.chain(args, kwargs.values())):
				# see analogous comment in distributionFunction
				return makeDelayedFunctionCall(helper, args, kwargs)
			else:
				return wrapped(instance, *args, **kwargs)
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

	@vectorOperator
	def offsetRotated(self, heading, offset) -> Vector:
		ro = offset.rotatedBy(heading)
		return self + ro

	@vectorOperator
	def offsetRadially(self, radius, heading) -> Vector:
		return self.offsetRotated(heading, Vector(0, radius))

	@scalarOperator
	def distanceTo(self, other) -> float:
		if not isinstance(other, Vector):
			return other.distanceTo(self)
		dx, dy = other.toVector() - self
		return math.hypot(dx, dy)

	@scalarOperator
	def angleTo(self, other) -> float:
		dx, dy = other.toVector() - self
		return normalizeAngle(math.atan2(dy, dx) - (math.pi / 2))

	@scalarOperator
	def angleWith(self, other) -> float:
		"""Compute the signed angle between self and other.

		The angle is positive if other is counterclockwise of self (considering
		the smallest possible rotation to align them).
		"""
		x, y = self.x, self.y
		ox, oy = other.x, other.y
		return normalizeAngle(math.atan2(oy, ox) - math.atan2(y, x))

	@scalarOperator
	def norm(self) -> float:
		return math.hypot(*self.coordinates)

	@vectorOperator
	def normalized(self) -> Vector:
		l = math.hypot(*self.coordinates)
		return Vector(*(coord/l for coord in self.coordinates))

	@vectorOperator
	def __add__(self, other) -> Vector:
		return Vector(self[0] + other[0], self[1] + other[1])

	@vectorOperator
	def __radd__(self, other) -> Vector:
		return Vector(self[0] + other[0], self[1] + other[1])

	@vectorOperator
	def __sub__(self, other) -> Vector:
		return Vector(self[0] - other[0], self[1] - other[1])

	@vectorOperator
	def __rsub__(self, other) -> Vector:
		return Vector(other[0] - self[0], other[1] - self[1])

	@vectorOperator
	def __mul__(self, other) -> Vector:
		return Vector(*(coord*other for coord in self.coordinates))

	def __rmul__(self, other) -> Vector:
		return self.__mul__(other)

	@vectorOperator
	def __truediv__(self, other) -> Vector:
		return Vector(*(coord/other for coord in self.coordinates))

	def __len__(self):
		return len(self.coordinates)

	def __getitem__(self, index):
		return self.coordinates[index]

	def __repr__(self):
		return f'({self.x} @ {self.y})'

	def __eq__(self, other):
		if isinstance(other, Vector):
			return other.coordinates == self.coordinates
		elif isinstance(other, (tuple, list)):
			return tuple(other) == self.coordinates
		else:
			return NotImplemented

	def __hash__(self):
		return hash(self.coordinates)

VectorDistribution._defaultValueType = Vector

class OrientedVector(Vector):
	def __init__(self, x, y, heading):
		super().__init__(x, y)
		self.heading = heading

	@staticmethod
	@distributionFunction
	def make(position, heading) -> OrientedVector:
		return OrientedVector(*position, heading)

	def toHeading(self):
		return self.heading

	def evaluateInner(self, context):
		hdg = valueInContext(self.heading, context)
		return OrientedVector(*(valueInContext(coord, context) for coord in self.coordinates), hdg)

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
