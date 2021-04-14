"""Support for checking Scenic types."""

import sys
import inspect
import numbers
import typing

from scenic.core.distributions import (Distribution, RejectionException, StarredDistribution,
                                       distributionFunction)
from scenic.core.lazy_eval import (DelayedArgument, valueInContext, requiredProperties,
                                   needsLazyEvaluation)
from scenic.core.vectors import Vector
from scenic.core.errors import RuntimeParseError, saveErrorLocation
from scenic.core.utils import get_type_origin, get_type_args

# Typing and coercion rules:
#
# coercible to a scalar:
#   instances of numbers.Real (by calling float())
# coercible to a heading:
#	anything coercible to a scalar
#	anything with a toHeading() method
# coercible to a Vector:
#   tuples/lists of length 2
#   anything with a toVector() method
# coercible to an object of type T:
#   instances of T
#
# Finally, Distributions are coercible to T iff their valueType is.

## Basic types

class Heading(float):
	"""Dummy class used as a target for type coercions to headings."""
	pass

def underlyingType(thing):
	"""What type this value ultimately evaluates to, if we can tell."""
	if isinstance(thing, Distribution):
		return thing.valueType
	elif isinstance(thing, TypeChecker) and len(thing.types) == 1:
		return thing.types[0]
	else:
		return type(thing)

def isA(thing, ty):
	"""Does this evaluate to a member of the given Scenic type?"""
	return issubclass(underlyingType(thing), ty)

def unifyingType(opts):		# TODO improve?
	"""Most specific type unifying the given types."""
	types = []
	for opt in opts:
		if isinstance(opt, StarredDistribution):
			ty = underlyingType(opt)
			typeargs = get_type_args(ty)
			if typeargs == ():
				types.append(ty)
			else:
				for ty in typeargs:
					if ty is not Ellipsis:
						types.append(ty)
		else:
			types.append(underlyingType(opt))
	if all(issubclass(ty, numbers.Real) for ty in types):
		return float
	mro = inspect.getmro(types[0])
	for parent in mro:
		if all(issubclass(ty, parent) for ty in types):
			return parent
	raise RuntimeError(f'broken MRO for types {types}')

## Type coercions (for internal use -- see the type checking API below)

def canCoerceType(typeA, typeB):
	"""Can values of typeA be coerced into typeB?"""
	import scenic.syntax.veneer as veneer	# TODO improve
	if get_type_origin(typeA) is typing.Union:
		# only raise an error now if none of the possible types will work;
		# we'll do more careful checking at runtime
		return any(canCoerceType(ty, typeB) for ty in get_type_args(typeA))
	if typeB is float:
		return issubclass(typeA, numbers.Real)
	elif typeB is Heading:
		return canCoerceType(typeA, float) or hasattr(typeA, 'toHeading')
	elif typeB is Vector:
		return issubclass(typeA, (tuple, list)) or hasattr(typeA, 'toVector')
	elif typeB is veneer.Behavior:
		return issubclass(typeA, typeB) or typeA in (type, type(None))
	else:
		return issubclass(typeA, typeB)

def canCoerce(thing, ty):
	"""Can this value be coerced into the given type?"""
	tt = underlyingType(thing)
	if canCoerceType(tt, ty):
		return True
	elif isinstance(thing, Distribution) and tt is object:
		return True		# fall back on type-checking at runtime
	else:
		return False

def coerce(thing, ty, error='wrong type'):
	"""Coerce something into the given type."""
	assert canCoerce(thing, ty), (thing, ty)

	import scenic.syntax.veneer as veneer	# TODO improve?
	realType = ty
	if ty is float:
		coercer = coerceToFloat
	elif ty is Heading:
		coercer = coerceToHeading
		ty = numbers.Real
		realType = float
	elif ty is Vector:
		coercer = coerceToVector
	elif ty is veneer.Behavior:
		coercer = coerceToBehavior
	else:
		coercer = None

	if isinstance(thing, Distribution):
		vt = thing.valueType
		if get_type_origin(vt) is typing.Union:
			possibleTypes = get_type_args(vt)
		else:
			possibleTypes = (vt,)
		if all(issubclass(possible, ty) for possible in possibleTypes):
			return thing 	# no coercion necessary
		else:
			return TypecheckedDistribution(thing, realType, error, coercer=coercer)
	elif coercer:
		try:
			return coercer(thing)
		except CoercionFailure as e:
			raise RuntimeParseError(f'{error} ({e.args[0]})') from None
	else:
		return thing

class CoercionFailure(Exception):
	pass

def coerceToFloat(thing) -> float:
	return float(thing)

def coerceToHeading(thing) -> float:
	if hasattr(thing, 'toHeading'):
		return thing.toHeading()
	return float(thing)

def coerceToVector(thing) -> Vector:
	if isinstance(thing, (tuple, list)):
		l = len(thing)
		if l != 2:
			raise CoercionFailure('expected 2D vector, got '
			                      f'{type(thing).__name__} of length {l}')
		return Vector(*thing)
	else:
		return thing.toVector()

def coerceToBehavior(thing):
	import scenic.syntax.veneer as veneer	# TODO improve
	if thing is None or isinstance(thing, veneer.Behavior):
		return thing
	else:
		assert issubclass(thing, veneer.Behavior)
		return thing()

class TypecheckedDistribution(Distribution):
	def __init__(self, dist, ty, errorMessage, coercer=None):
		super().__init__(dist, valueType=ty)
		self.dist = dist
		self.errorMessage = errorMessage
		self.coercer = coercer
		self.loc = saveErrorLocation()

	def sampleGiven(self, value):
		val = value[self.dist]
		suffix = None
		if self.coercer:
			if canCoerceType(type(val), self.valueType):
				try:
					return self.coercer(val)
				except CoercionFailure as e:
					suffix = f' ({e.args[0]})'
		elif isinstance(val, self.valueType):
			return val
		if suffix is None:
			suffix = f' (expected {self.valueType.__name__}, got {type(val).__name__})'
		raise RuntimeParseError(self.errorMessage + suffix, self.loc)

	def conditionTo(self, value):
		self.dist.conditionTo(value)

	def __repr__(self):
		return f'TypecheckedDistribution({self.dist}, {self.valueType})'

def coerceToAny(thing, types, error):
	"""Coerce something into any of the given types, printing an error if impossible."""
	for ty in types:
		if canCoerce(thing, ty):
			return coerce(thing, ty, error)
	from scenic.syntax.veneer import verbosePrint
	verbosePrint(f'Failed to coerce {thing} of type {underlyingType(thing)} to {types}',
	             file=sys.stderr)
	raise RuntimeParseError(error)

## Top-level type checking/conversion API

def toTypes(thing, types, typeError='wrong type'):
	"""Convert something to any of the given types, printing an error if impossible."""
	if needsLazyEvaluation(thing):
		# cannot check the type now; create proxy object to check type after evaluation
		return TypeChecker(thing, types, typeError)
	else:
		return coerceToAny(thing, types, typeError)

def toType(thing, ty, typeError='wrong type'):
	"""Convert something to a given type, printing an error if impossible."""
	return toTypes(thing, (ty,), typeError)

def toScalar(thing, typeError='non-scalar in scalar context'):
	"""Convert something to a scalar, printing an error if impossible."""
	return toType(thing, float, typeError)

def toHeading(thing, typeError='non-heading in heading context'):
	"""Convert something to a heading, printing an error if impossible."""
	return toType(thing, Heading, typeError)

def toVector(thing, typeError='non-vector in vector context'):
	"""Convert something to a vector, printing an error if impossible."""
	return toType(thing, Vector, typeError)

def evaluateRequiringEqualTypes(func, thingA, thingB, typeError='type mismatch'):
	"""Evaluate the func, assuming thingA and thingB have the same type.

	If func produces a lazy value, it should not have any required properties beyond
	those of thingA and thingB."""
	if not needsLazyEvaluation(thingA) and not needsLazyEvaluation(thingB):
		if underlyingType(thingA) is not underlyingType(thingB):
			raise RuntimeParseError(typeError)
		return func()
	else:
		# cannot check the types now; create proxy object to check types after evaluation
		return TypeEqualityChecker(func, thingA, thingB, typeError)

## Proxy objects for lazy type checking

class TypeChecker(DelayedArgument):
	"""Checks that a given lazy value has one of a given list of types."""
	def __init__(self, arg, types, error):
		def check(context):
			val = arg.evaluateIn(context)
			return coerceToAny(val, types, error)
		super().__init__(requiredProperties(arg), check)
		self.inner = arg
		self.types = types

	def __str__(self):
		return f'TypeChecker({self.inner},{self.types})'

class TypeEqualityChecker(DelayedArgument):
	"""Lazily evaluates a function, after checking that two lazy values have the same type."""
	def __init__(self, func, checkA, checkB, error):
		props = requiredProperties(checkA) | requiredProperties(checkB)
		def check(context):
			ca = valueInContext(checkA, context)
			cb = valueInContext(checkB, context)
			if underlyingType(ca) is not underlyingType(cb):
				raise RuntimeParseError(error)
			return valueInContext(func(), context)
		super().__init__(props, check)
		self.inner = func
		self.checkA = checkA
		self.checkB = checkB

	def __str__(self):
		return f'TypeEqualityChecker({self.inner},{self.checkA},{self.checkB})'
