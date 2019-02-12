
### Distributions

import collections
import itertools
import random

from scenic.core.utils import argsToString, RuntimeParseError

## Misc

def valueInContext(value, context):
	try:
		return value.evaluateIn(context)
	except AttributeError:
		return value

def dependencies(thing):
	return getattr(thing, '_dependencies', ())

def needsSampling(thing):
	return isinstance(thing, Distribution) or dependencies(thing)

def supportInterval(thing):
	if hasattr(thing, 'supportInterval'):
		return thing.supportInterval()
	elif isinstance(thing, (int, float)):
		return thing, thing
	else:
		return None, None

def underlyingFunction(thing):
	return getattr(thing, '_underlyingFunction', thing)

class RejectionException(Exception):
	pass

## Abstract distributions

class DefaultIdentityDict(dict):
	def __getitem__(self, key):
		if not isinstance(key, Samplable):		# to allow non-hashable objects
			return key
		return super().__getitem__(key)

	def __missing__(self, key):
		return key

class Samplable:
	def __init__(self, dependencies):
		deps = []
		for dep in dependencies:
			if needsSampling(dep):
				deps.append(dep)
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
		if subsamples is None:
			subsamples = DefaultIdentityDict()
		for child in self._conditioned._dependencies:
			if child not in subsamples:
				subsamples[child] = child.sample(subsamples)
		return self._conditioned.sampleGiven(subsamples)

	def sampleGiven(self, value):
		return DefaultIdentityDict({ dep: value[dep] for dep in self._dependencies })

	def conditionTo(self, value):
		assert isinstance(value, Samplable)
		self._conditioned = value

class Distribution(Samplable):
	"""Abstract class for distributions"""

	defaultValueType = float

	def __init__(self, *dependencies, valueType=None):
		props = set()
		for dep in dependencies:
			if hasattr(dep, 'requiredProperties'):
				props.update(dep.requiredProperties)
		super().__init__(dependencies)
		self.requiredProperties = props
		if valueType is None:
			valueType = self.defaultValueType
		self.valueType = valueType

	def clone(self):
		raise NotImplementedError('clone() not supported by this distribution')

	def evaluateIn(self, context):
		if self.requiredProperties:
			self.evaluateInner(context)
			self.requiredProperties = set()
		return self

	def evaluateInner(self, context):
		pass

	def supportInterval(self):
		return None, None

	def __getattr__(self, name):
		return AttributeDistribution(name, self)

	def dependencyTree(self):
		l = [str(self)]
		for dep in self.dependencies:
			for line in dep.dependencyTree():
				l.append('  ' + line)
		return l

## Derived distributions

class CustomDistribution(Distribution):
	"""Distribution with a custom sampler given by an arbitrary function"""
	def __init__(self, sampler, *dependencies, name='CustomDistribution', evaluator=None):
		super(CustomDistribution, self).__init__(*dependencies)
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
		return f'{self.name}{argsToString(self.dependencies)}'

class TupleDistribution(Distribution, collections.abc.Sequence):
	"""Distributions over tuples"""
	def __init__(self, *coordinates):
		super(TupleDistribution, self).__init__(*coordinates)
		self.coordinates = coordinates

	def __len__(self):
		return len(self.coordinates)

	def __getitem__(self, index):
		return self.coordinates[index]

	def sampleGiven(self, value):
		return tuple(value[coordinate] for coordinate in self.coordinates)

	def evaluateInner(self, context):
		self.coordinates = tuple(valueInContext(coord, context) for coord in self.coordinates)

	def __str__(self):
		coords = ', '.join(str(c) for c in self.coordinates)
		return f'({coords})'

def toDistribution(val, always=True):
	if type(val) is tuple:
		coords = [toDistribution(c, always=True) for c in val]
		needed = always or any(isinstance(c, Distribution) for c in coords)
		return TupleDistribution(*coords) if needed else val
	return val

class FunctionDistribution(Distribution):
	"""Distribution resulting from passing distributions to a function"""
	def __init__(self, func, args, support=None):
		args = tuple(toDistribution(arg) for arg in args)
		super(FunctionDistribution, self).__init__(*args)
		self.function = func
		self.arguments = args
		self.support = support

	def sampleGiven(self, value):
		args = tuple(value[arg] for arg in self.arguments)
		return self.function(*args)

	def evaluateInner(self, context):
		self.function = valueInContext(self.function, context)
		self.arguments = tuple(valueInContext(arg, context) for arg in self.arguments)

	def supportInterval(self):
		if self.support is None:
			return None, None
		subsupports = (supportInterval(arg) for arg in self.arguments)
		return self.support(*subsupports)

	def __str__(self):
		return f'{self.function.__name__}{argsToString(self.arguments)}'

def distributionFunction(method, support=None):
	def helper(*args):
		args = tuple(toDistribution(arg, always=False) for arg in args)
		if any(isinstance(arg, Distribution) for arg in args):
			return FunctionDistribution(method, args, support)
		else:
			return method(*args)
	helper._underlyingFunction = method
	return helper

def monotonicDistributionFunction(method):
	def support(*subsupports):
		mins, maxes = zip(*subsupports)
		l = None if None in mins else method(*mins)
		r = None if None in maxes else method(*maxes)
		return l, r
	return distributionFunction(method, support=support)

class MethodDistribution(Distribution):
	"""Distribution resulting from passing distributions to a method of a fixed object"""
	def __init__(self, method, obj, args):
		args = tuple(toDistribution(arg) for arg in args)
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
		return f'{self.object}.{self.method.__name__}{argsToString(self.arguments)}'

def distributionMethod(method):
	def helper(self, *args):
		args = tuple(toDistribution(arg, always=False) for arg in args)
		if any(needsSampling(arg) for arg in args):
			return MethodDistribution(method, self, args)
		else:
			return method(self, *args)
	helper._underlyingFunction = method
	return helper

class AttributeDistribution(Distribution):
	"""Distribution resulting from accessing an attribute of a distribution"""
	def __init__(self, attribute, obj):
		super(AttributeDistribution, self).__init__(obj)
		self.attribute = attribute
		self.object = obj

	def sampleGiven(self, value):
		obj = value[self.object]
		return getattr(obj, self.attribute)

	def evaluateInner(self, context):
		self.object = valueInContext(self.object, context)

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
		self.object = valueInContext(self.object, context)
		self.operands = tuple(valueInContext(arg, context) for arg in self.operands)

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
		self.low = valueInContext(self.low, context)
		self.high = valueInContext(self.high, context)

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
		self.mean = valueInContext(self.mean, context)
		self.stddev = valueInContext(self.stddev, context)

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
					raise RuntimeParseError(f'discrete distribution weight {prob} is not a constant number')
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
		self.options = [valueInContext(opt, context) for opt in self.options]

	def __str__(self):
		if self.cumulativeWeights is not None:
			return f'Options({self.weights})'
		else:
			return f'Options{argsToString(self.options)}'
