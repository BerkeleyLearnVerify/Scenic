
### Distributions

import collections
import itertools
import random

from scenic.core.lazy_eval import (LazilyEvaluable, DelayedArgument,
                                   requiredProperties, needsLazyEvaluation, valueInContext,
                                   makeDelayedFunctionCall)
from scenic.core.utils import argsToString, RuntimeParseError

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
	return getattr(thing, '_underlyingFunction', thing)

class RejectionException(Exception):
	"""Exception used to signal that the sample currently being generated must be rejected."""
	pass

## Abstract distributions

class DefaultIdentityDict(dict):
	"""Dictionary which is the identity map by default."""
	def __getitem__(self, key):
		if not isinstance(key, Samplable):		# to allow non-hashable objects
			return key
		return super().__getitem__(key)

	def __missing__(self, key):
		return key

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
		return { q: subsamples[q] for q in quantities }

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
		assert isinstance(value, Samplable)
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

class Distribution(Samplable):
	"""Abstract class for distributions."""

	defaultValueType = float

	def __init__(self, *dependencies, valueType=None):
		super().__init__(dependencies)
		if valueType is None:
			valueType = self.defaultValueType
		self.valueType = valueType

	def clone(self):
		"""Construct an independent copy of this Distribution."""
		raise NotImplementedError('clone() not supported by this distribution')

	def supportInterval(self):
		"""Compute lower and upper bounds on the value of this Distribution."""
		return None, None

	def __getattr__(self, name):
		return AttributeDistribution(name, self)

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

	def sampleGiven(self, value):
		return self.builder(value[coordinate] for coordinate in self.coordinates)

	def evaluateInner(self, context):
		coordinates = (valueInContext(coord, context) for coord in self.coordinates)
		return TupleDistribution(*coordinates, builder=self.builder)

	def __str__(self):
		coords = ', '.join(str(c) for c in self.coordinates)
		return f'({coords}, builder={self.builder})'

def toDistribution(val):
	"""Wrap Python data types with Distributions, if necessary.

	For example, tuples containing Samplables need to be converted into TupleDistributions
	in order to keep track of dependencies properly."""
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
	def __init__(self, func, args, kwargs, support=None):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		super().__init__(*args, *kwargs.values())
		self.function = func
		self.arguments = args
		self.kwargs = kwargs
		self.support = support

	def sampleGiven(self, value):
		args = tuple(value[arg] for arg in self.arguments)
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

	def __str__(self):
		args = argsToString(itertools.chain(self.arguments, self.kwargs.items()))
		return f'{self.function.__name__}{args}'

def distributionFunction(method, support=None):
	"""Decorator for wrapping a function so that it can take distributions as arguments."""
	def helper(*args, **kwargs):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
			return FunctionDistribution(method, args, kwargs, support)
		elif any(needsLazyEvaluation(arg) for arg in itertools.chain(args, kwargs.values())):
			# recursively call this helper (not the original function), since the delayed
			# arguments may evaluate to distributions, in which case we'll have to make a
			# FunctionDistribution
			return makeDelayedFunctionCall(helper, args, kwargs)
		else:
			return method(*args, **kwargs)
	helper._underlyingFunction = method
	return helper

def monotonicDistributionFunction(method):
	"""Like distributionFunction, but additionally specifies that the function is monotonic."""
	def support(*subsupports, **kwss):
		mins, maxes = zip(*subsupports)
		kwmins = { name: interval[0] for name, interval in kwss.items() }
		kwmaxes = { name: interval[1] for name, interval in kwss.items() }
		l = None if None in mins or None in kwmins else method(*mins, **kwmins)
		r = None if None in maxes or None in kwmaxes else method(*maxes, **kwmaxes)
		return l, r
	return distributionFunction(method, support=support)

class MethodDistribution(Distribution):
	"""Distribution resulting from passing distributions to a method of a fixed object"""
	def __init__(self, method, obj, args, kwargs):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
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
		return MethodDistribution(self.method, obj, arguments, kwargs)

	def __str__(self):
		args = argsToString(itertools.chain(self.arguments, self.kwargs.items()))
		return f'{self.object}.{self.method.__name__}{args}'

def distributionMethod(method):
	"""Decorator for wrapping a method so that it can take distributions as arguments."""
	def helper(self, *args, **kwargs):
		args = tuple(toDistribution(arg) for arg in args)
		kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
		if any(needsSampling(arg) for arg in itertools.chain(args, kwargs.values())):
			return MethodDistribution(method, self, args, kwargs)
		elif any(needsLazyEvaluation(arg) for arg in itertools.chain(args, kwargs.values())):
			# see analogous comment in distributionFunction
			return makeDelayedFunctionCall(helper, (self,) + args, kwargs)
		else:
			return method(self, *args, **kwargs)
	helper._underlyingFunction = method
	return helper

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

	def __str__(self):
		return f'{self.object}.{self.attribute}'

class OperatorDistribution(Distribution):
	"""Distribution resulting from applying an operator to one or more distributions"""
	def __init__(self, operator, obj, operands):
		operands = tuple(toDistribution(arg) for arg in operands)
		super().__init__(obj, *operands)
		self.operator = operator
		self.object = obj
		self.operands = operands

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

	def __str__(self):
		return f'{self.object}.{self.operator}{argsToString(self.operands)}'

# Operators which can be applied to distributions.
# Note that we deliberately do not include comparisons and __bool__,
# since Scenic does not allow control flow to depend on random variables.
allowedOperators = [
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
	'__len__',
	'__getitem__',
	'__call__'
]
def makeOperatorHandler(op):
	def handler(self, *args):
		return OperatorDistribution(op, self, args)
	return handler
for op in allowedOperators:
	setattr(Distribution, op, makeOperatorHandler(op))

## Simple distributions

import scenic.core.type_support as type_support

class Range(Distribution):
	"""Uniform distribution over a range"""
	def __init__(self, low, high):
		low = type_support.toScalar(low, f'Range endpoint {low} is not a scalar')
		high = type_support.toScalar(high, f'Range endpoint {high} is not a scalar')
		super().__init__(low, high)
		self.low = low
		self.high = high

	def __contains__(self, obj):
		return low <= obj and obj <= high

	def clone(self):
		return type(self)(self.low, self.high)

	def sampleGiven(self, value):
		return random.uniform(value[self.low], value[self.high])

	def evaluateInner(self, context):
		low = valueInContext(self.low, context)
		high = valueInContext(self.high, context)
		return Range(low, high)

	def __str__(self):
		return f'Range({self.low}, {self.high})'

class Normal(Distribution):
	"""Normal distribution"""
	def __init__(self, mean, stddev):
		mean = type_support.toScalar(mean, f'Normal mean {mean} is not a scalar')
		stddev = type_support.toScalar(stddev, f'Normal stddev {stddev} is not a scalar')
		super().__init__(mean, stddev)
		self.mean = mean
		self.stddev = stddev

	def clone(self):
		return type(self)(self.mean, self.stddev)

	def sampleGiven(self, value):
		return random.gauss(value[self.mean], value[self.stddev])

	def evaluateInner(self, context):
		mean = valueInContext(self.mean, context)
		stddev = valueInContext(self.stddev, context)
		return Normal(mean, stddev)

	def __str__(self):
		return f'Normal({self.mean}, {self.stddev})'

class Options(Distribution):
	"""Distribution over a finite list of options.

	Specified by a dict giving probabilities; otherwise uniform over a given iterable.
	"""
	def __init__(self, opts):
		if isinstance(opts, dict):
			self.options = []
			self.weights = dict()
			ordered = []
			for opt, prob in opts.items():
				if not isinstance(prob, (float, int)):
					raise RuntimeParseError(f'discrete distribution weight {prob}'
					                        ' is not a constant number')
				if prob < 0:
					raise RuntimeParseError(f'discrete distribution weight {prob} is negative')
				if prob == 0:
					continue
				opt = toDistribution(opt)
				self.options.append(opt)
				self.weights[opt] = prob
				ordered.append(prob)
			self.cumulativeWeights = tuple(itertools.accumulate(ordered))
		else:
			self.options = tuple(toDistribution(opt) for opt in opts)
			self.cumulativeWeights = None
		if len(self.options) == 0:
			raise RuntimeParseError('tried to make discrete distribution over empty domain!')
		valueType = type_support.unifyingType(self.options)
		super().__init__(*self.options, valueType=valueType)

	def clone(self):
		return type(self)(self.weights if self.cumulativeWeights is not None else self.options)

	def sampleGiven(self, value):
		opts = [value[opt] for opt in self.options]
		return random.choices(opts, cum_weights=self.cumulativeWeights)[0]

	def evaluateInner(self, context):
		if self.cumulativeWeights is not None:
			return Options({ valueInContext(opt, context): wt
			               for opt, wt in self.weights.items() })
		else:
			return Options([valueInContext(opt, context) for opt in self.options])

	def __str__(self):
		if self.cumulativeWeights is not None:
			return f'Options({self.weights})'
		else:
			return f'Options{argsToString(self.options)}'
