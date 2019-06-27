
### Support for checking Scenic types

import sys
import inspect

import numpy as np

from scenic.core.distributions import Distribution
from scenic.core.lazy_eval import (DelayedArgument, valueInContext, requiredProperties,
                                   needsLazyEvaluation, toDelayedArgument)
from scenic.core.vectors import Vector
from scenic.core.utils import RuntimeParseError

# Typing and coercion rules:
#
# coercible to a scalar:
#   float
#   int (by conversion to float)
#	numpy real scalar types (likewise)
# coercible to a heading:
#	anything coercible to a scalar
#	anything with a toHeading() method
# coercible to a Vector:
#   anything with a toVector() method
# coercible to an object of type T:
#   instances of T
#
# Finally, Distributions are coercible to T iff their valueType is.

## Basic types

class Heading:
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
	types = [underlyingType(opt) for opt in opts]
	if all(issubclass(ty, (float, int)) for ty in types):
		return float
	mro = inspect.getmro(types[0])
	for parent in mro:
		if all(issubclass(ty, parent) for ty in types):
			return parent
	raise RuntimeError(f'broken MRO for types {types}')

## Type coercions (for internal use -- see the type checking API below)

def canCoerceType(typeA, typeB):
	"""Can values of typeA be coerced into typeB?"""
	if typeB is float:
		if issubclass(typeA, (float, int)):
			return True
		if issubclass(typeA, np.number) and not issubclass(typeA, np.complexfloating):
			return True
		return False
	elif typeB is Heading:
		return canCoerceType(typeA, float) or hasattr(typeA, 'toHeading')
	elif typeB is Vector:
		return hasattr(typeA, 'toVector')
	else:
		return issubclass(typeA, typeB)

def canCoerce(thing, ty):
	"""Can this value be coerced into the given type?"""
	tt = underlyingType(thing)
	return canCoerceType(tt, ty)

def coerce(thing, ty):
	"""Coerce something into the given type."""
	assert canCoerce(thing, ty), (thing, ty)
	if isinstance(thing, Distribution):
		return thing
	if ty is float:
		return float(thing)
	elif ty is Heading:
		return thing.toHeading() if hasattr(thing, 'toHeading') else float(thing)
	elif ty is Vector:
		return thing.toVector()
	else:
		return thing

def coerceToAny(thing, types, error):
	"""Coerce something into any of the given types, printing an error if impossible."""
	for ty in types:
		if canCoerce(thing, ty):
			return coerce(thing, ty)
	print(f'Failed to coerce {thing} of type {underlyingType(thing)} to {types}', file=sys.stderr)
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
