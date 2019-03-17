
### Vectors

import math
from math import sin, cos
import random
import collections

import shapely.geometry

from scenic.core.distributions import Samplable, Distribution, MethodDistribution
from scenic.core.distributions import needsSampling, makeOperatorHandler
from scenic.core.distributions import distributionMethod
from scenic.core.specifiers import valueInContext
import scenic.core.utils as utils
from scenic.core.geometry import normalizeAngle

class VectorDistribution(Distribution):
	defaultValueType = None		# will be set after Vector is defined

	def toVector(self):
		return self

class CustomVectorDistribution(VectorDistribution):
	"""Distribution with a custom sampler given by an arbitrary function"""
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
		self.evaluator(self, context)

	def __str__(self):
		deps = utils.argsToString(self.dependencies)
		return f'{self.name}{deps}'

class VectorOperatorDistribution(VectorDistribution):
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
		self.object = valueInContext(self.object, context)
		self.operands = tuple(valueInContext(arg, context) for arg in self.operands)

	def __str__(self):
		ops = utils.argsToString(self.operands)
		return f'{self.object}.{self.operator}{ops}'

class VectorMethodDistribution(VectorDistribution):
	def __init__(self, method, obj, args):
		super().__init__(*args)
		self.method = method
		self.object = obj
		self.arguments = args

	def sampleGiven(self, value):
		args = [value[arg] for arg in self.arguments]
		return self.method(self.object, *args)

	def evaluateInner(self, context):
		self.object = valueInContext(self.object, context)
		self.arguments = tuple(valueInContext(arg, context) for arg in self.arguments)

	def __str__(self):
		args = utils.argsToString(self.arguments)
		return f'{self.object}.{self.method.__name__}{args}'

def scalarOperator(method):
	op = method.__name__
	setattr(VectorDistribution, op, makeOperatorHandler(op))
	def handler2(self, *args):
		if any(needsSampling(arg) for arg in args):
			return MethodDistribution(method, self, args)
		else:
			return method(self, *args)
	return handler2

def makeVectorOperatorHandler(op):
	def handler(self, *args):
		return VectorOperatorDistribution(op, self, args)
	return handler
def vectorOperator(method):
	op = method.__name__
	setattr(VectorDistribution, op, makeVectorOperatorHandler(op))
	def handler2(self, *args):
		if needsSampling(self):
			return VectorOperatorDistribution(op, self, args)
		elif any(needsSampling(arg) for arg in args):
			return VectorMethodDistribution(method, self, args)
		else:
			return method(self, *args)
	return handler2

def vectorDistributionMethod(method):
	def helper(self, *args):
		if any(needsSampling(arg) for arg in args):
			return VectorMethodDistribution(method, self, args)
		else:
			return method(self, *args)
	return helper

class Vector(Samplable, collections.abc.Sequence):
	def __init__(self, x, y):
		self.coordinates = (x, y)
		super().__init__(self.coordinates)

	@property
	def x(self):
		return self.coordinates[0]

	@property
	def y(self):
		return self.coordinates[1]

	def toVector(self):
		return self

	def sampleGiven(self, value):
		return Vector(*(value[coord] for coord in self.coordinates))

	@vectorOperator
	def rotatedBy(self, angle):
		x, y = self.x, self.y
		c, s = cos(angle), sin(angle)
		return Vector((c * x) - (s * y), (s * x) + (c * y))

	@vectorOperator
	def offsetRotated(self, heading, offset):
		ro = offset.rotatedBy(heading)
		return self + ro

	@vectorOperator
	def offsetRadially(self, radius, heading):
		return self.offsetRotated(heading, Vector(0, radius))

	@scalarOperator
	def distanceTo(self, other):
		dx, dy = other.toVector() - self
		return math.hypot(dx, dy)

	@scalarOperator
	def angleTo(self, other):
		dx, dy = other.toVector() - self
		return normalizeAngle(math.atan2(dy, dx) - (math.pi / 2))

	@vectorOperator
	def __add__(self, other):
		return Vector(self[0] + other[0], self[1] + other[1])

	@vectorOperator
	def __radd__(self, other):
		return Vector(self[0] + other[0], self[1] + other[1])

	@vectorOperator
	def __sub__(self, other):
		return Vector(self[0] - other[0], self[1] - other[1])

	@vectorOperator
	def __rsub__(self, other):
		return Vector(other[0] - self[0], other[1] - self[1])

	def __len__(self):
		return len(self.coordinates)

	def __getitem__(self, index):
		return self.coordinates[index]

	def __repr__(self):
		return f'({self.x} @ {self.y})'

VectorDistribution.defaultValueType = Vector

class OrientedVector(Vector):
	def __init__(self, x, y, heading):
		super().__init__(x, y)
		self.heading = heading

	def toHeading(self):
		return self.heading

class VectorField:
	def __init__(self, name, value):
		self.name = name
		self.value = value
		self.valueType = float

	@distributionMethod
	def __getitem__(self, pos):
		return self.value(pos)

	@vectorDistributionMethod
	def followFrom(self, pos, dist, steps=4):
		step = dist / steps
		for i in range(steps):
			pos = pos.offsetRadially(step, self[pos])
		return pos

	def __str__(self):
		return f'<{type(self).__name__} {self.name}>'

class PolygonalVectorField(VectorField):
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
		raise RuntimeError(f'evaluated PolygonalVectorField at undefined point {pos}')
