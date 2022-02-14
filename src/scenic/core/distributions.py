"""Objects representing distributions that can be sampled from."""

import collections
import itertools
import random
import math
import typing
import warnings

import numpy
import decorator

from scenic.core.lazy_eval import (LazilyEvaluable,
    requiredProperties, needsLazyEvaluation, valueInContext, makeDelayedFunctionCall)
from scenic.core.utils import DefaultIdentityDict, argsToString, areEquivalent, cached, sqrt2
from scenic.core.errors import RuntimeParseError

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

class Samplable(LazilyEvaluable):
	"""Abstract class for values which can be sampled, possibly depending on other values.

	Samplables may specify a proxy object which must have the same distribution as the
	original after conditioning on the scenario's requirements. This allows transparent
	conditioning without modifying Samplable fields of immutable objects.

	Args:
		dependencies: sequence of values that this value may depend on (formally, objects
			for which sampled values must be provided to `sampleGiven`). It is legal to
			include values which are not instances of `Samplable`, e.g. integers.

	Attributes:
		_conditioned: proxy object as described above; set using `conditionTo`.
		_dependencies: tuple of other samplables which must be sampled before this one;
			set by the initializer and subsequently immutable.
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

		Args:
			value (DefaultIdentityDict): dictionary mapping objects to their sampled
				values. Guaranteed to provide values for all objects given in the set of
				dependencies when this `Samplable` was created.
		"""
		return DefaultIdentityDict({ dep: value[dep] for dep in self._dependencies })

	def conditionTo(self, value):
		"""Condition this value to another value with the same conditional distribution."""
		assert isinstance(value, Samplable)
		self._conditioned = value

	def evaluateIn(self, context):
		"""See `LazilyEvaluable.evaluateIn`."""
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

class ConstantSamplable(Samplable):
	"""A samplable which always evaluates to a constant value.

	Only for internal use.
	"""
	def __init__(self, value):
		assert not needsSampling(value)
		assert not needsLazyEvaluation(value)
		self.value = value
		super().__init__(())

	def sampleGiven(self, value):
		return self.value

class Distribution(Samplable):
	"""Abstract class for distributions.

	.. note::

		When called during dynamic simulations (vs. scenario compilation), constructors
		for distributions return *actual sampled values*, not `Distribution` objects.

	Args:
		dependencies: values which this distribution may depend on (see `Samplable`).
		valueType: **_valueType** to use (see below), or `None` for the default.

	Attributes:
		_valueType: type of the values sampled from this distribution, or `object` if the
			type is not known.
	"""

	#: Default valueType for distributions of this class, when not otherwise specified.
	_defaultValueType = object

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
			valueType = self._defaultValueType
		self._valueType = valueType

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
		`clone`.
		"""
		raise NotImplementedError('bucket() not supported by this distribution')

	def supportInterval(self):
		"""Compute lower and upper bounds on the value of this Distribution.

		By default returns :samp:`(None, None)` indicating that no lower or upper bounds
		are known. Subclasses may override this method to provide more accurate results.
		"""
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
	def helper(wrapped, *args, **kwargs):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
			return FunctionDistribution(wrapped, args, kwargs, support, valueType)
		elif any(needsLazyEvaluation(arg)
		         for arg in itertools.chain(args, kwargs.values())):
			# recursively call this helper (not the original function), since the
			# delayed arguments may evaluate to distributions, in which case we'll
			# have to make a FunctionDistribution
			return makeDelayedFunctionCall(helper, (wrapped,) + args, kwargs)
		else:
			return wrapped(*args, **kwargs)
	return unpacksDistributions(decorator.decorate(wrapped, helper, kwsyntax=True))

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
		super().__init__(value, valueType=value._valueType)

	def sampleGiven(self, value):
		return value[self.value]

	def evaluateInner(self, context):
		return StarredDistribution(valueInContext(self.value, context), self.lineno)

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
	def helper(wrapped, self, *args, **kwargs):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
			return MethodDistribution(method, self, args, kwargs)
		elif any(needsLazyEvaluation(arg)
		         for arg in itertools.chain(args, kwargs.values())):
			# see analogous comment in distributionFunction
			return makeDelayedFunctionCall(helper, (method, self) + args, kwargs)
		else:
			return method(self, *args, **kwargs)
	return unpacksDistributions(decorator.decorate(method, helper, kwsyntax=True))

class AttributeDistribution(Distribution):
	"""Distribution resulting from accessing an attribute of a distribution"""
	def __init__(self, attribute, obj):
		super().__init__(obj)
		self.attribute = attribute
		self.object = obj

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
		vty = self.object._valueType
		retTy = None
		if vty is not object:
			func = getattr(vty, self.attribute, None)
			if func:
				if isinstance(func, property):
					func = func.fget
				retTy = typing.get_type_hints(func).get('return')
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

	@staticmethod
	def inferType(obj, operator):
		if issubclass(obj._valueType, (float, int)):
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

	def __contains__(self, obj):
		return low <= obj and obj <= high

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
			raise RejectionException('tried to make discrete distribution over empty domain!')

		index = self.makeSelector(len(options)-1, weights)
		super().__init__(index, options)

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
