
### Support for checking Scenic types

import inspect

import numpy as np

from scenic.core.distributions import Distribution
from scenic.core.specifiers import DelayedArgument, valueInContext
from scenic.core.vectors import Vector
from scenic.core.utils import RuntimeParseError

# Typing and coercion rules:
#
# coercible to a scalar:
#   float
#   int (by conversion to float)
# coercible to a heading:
#	anything coercible to a scalar
#	anything with a toHeading() method
# coercible to a Vector:
#   anything with a toVector() method
# coercible to an object of type T:
#   instances of T
#
# Finally, Distributions are coercible to T iff their valueType is.

class Heading:
	"""Dummy class used as a target for type coercions to headings."""
	pass

def underlyingType(thing):
	if isinstance(thing, Distribution):
		return thing.valueType
	elif isinstance(thing, TypeChecker) and len(thing.types) == 1:
		return thing.types[0]
	else:
		return type(thing)

def isA(thing, ty):
	return issubclass(underlyingType(thing), ty)

def unifyingType(opts):		# TODO improve?
	types = [underlyingType(opt) for opt in opts]
	if all(issubclass(ty, (float, int)) for ty in types):
		return float
	mro = inspect.getmro(types[0])
	for parent in mro:
		if all(issubclass(ty, parent) for ty in types):
			return parent
	raise RuntimeError(f'broken MRO for types {types}')

def canCoerceType(typeA, typeB):
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
	tt = underlyingType(thing)
	return canCoerceType(tt, ty)

def coerce(thing, ty):
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
	for ty in types:
		if canCoerce(thing, ty):
			return coerce(thing, ty)
	print(f'Failed to coerce {thing} to {types}')
	raise RuntimeParseError(error)

def toTypes(thing, types, typeError='wrong type'):
	if isinstance(thing, DelayedArgument):
		return TypeChecker(thing, types, typeError)
	else:
		return coerceToAny(thing, types, typeError)

def toType(thing, ty, typeError='wrong type'):
	return toTypes(thing, (ty,), typeError)

def toScalar(thing, typeError='non-scalar in scalar context'):
	return toType(thing, float, typeError)

def toHeading(thing, typeError='non-heading in heading context'):
	return toType(thing, Heading, typeError)

def toVector(thing, typeError='non-vector in vector context'):
	return toType(thing, Vector, typeError)

def valueRequiringEqualTypes(val, thingA, thingB, typeError='type mismatch'):
	if not isinstance(thingA, DelayedArgument) and not isinstance(thingB, DelayedArgument):
		if underlyingType(thingA) is not underlyingType(thingB):
			raise RuntimeParseError(typeError)
		return val
	else:
		return TypeEqualityChecker(val, thingA, thingB, typeError)

class TypeChecker(DelayedArgument):
	def __init__(self, arg, types, error):
		def check(context):
			val = arg.evaluateIn(context)
			return coerceToAny(val, types, error)
		super().__init__(arg.requiredProperties, check)
		self.inner = arg
		self.types = types

	def __str__(self):
		return f'TypeChecker({self.inner},{self.types})'

class TypeEqualityChecker(DelayedArgument):
	def __init__(self, arg, checkA, checkB, error):
		arg = toDelayedArgument(arg)
		assert requiredProperties(checkA) <= arg.requiredProperties
		assert requiredProperties(checkB) <= arg.requiredProperties
		def check(context):
			ca = valueInContext(checkA, context)
			cb = valueInContext(checkB, context)
			assert not requiredProperties(ca) and not requiredProperties(cb)
			if underlyingType(ca) is not underlyingType(cb):
				raise RuntimeParseError(error)
			return arg.evaluateIn(context)
		super().__init__(arg.requiredProperties, check)
		self.inner = arg
		self.checkA = checkA
		self.checkB = checkB

	def __str__(self):
		return f'TypeEqualityChecker({self.inner},{self.checkA},{self.checkB})'
