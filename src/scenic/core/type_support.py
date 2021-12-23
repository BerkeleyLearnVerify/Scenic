"""Support for checking Scenic types.

This module provides a system for checking that values passed to Scenic operators and
functions have the expected types. The top-level function `toTypes` and its
specializations `toType`, `toVector`, `toScalar`, etc. can also *coerce* closely-related
types into the desired type in some cases. For lazily-evaluated values (random values and
delayed arguments of specifiers), it may not be possible to determine the type at object
creation time: in such cases these functions return a lazily-evaluated object that
performs the type check either during specifier resolution or sampling as needed.

In general, the only objects which are coercible to a type T are instances of that type,
together with `Distribution` objects whose **_valueType** is a type coercible to T (and
therefore whose sampled value can be coerced to T). However, we also have the following
exceptional rules:

	* Coercible to a scalar (type `float`):
		* Instances of `numbers.Real` (coerced by calling `float` on them);
		  this includes NumPy types such as `numpy.single`
	* Coercible to a heading (type `Heading`):
		* Anything coercible to a scalar
		* Any type with a **toHeading** method (including `OrientedPoint`)
	* Coercible to a vector (type `Vector`):
		* Tuples and lists of length 2
		* Any type with a **toVector** method (including `Point`)
	* Coercible to a `Behavior`:
		* Subclasses of `Behavior` (coerced by calling them with no arguments)
		* `None` (considered to have type `Behavior` for convenience)
"""

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

## Basic types

class Heading(float):
	"""Dummy class used as a target for type coercions to headings."""
	pass

def underlyingType(thing):
	"""What type this value ultimately evaluates to, if we can tell."""
	if isinstance(thing, Distribution):
		return thing._valueType
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
	"""Coerce something into the given type.

	Used internally by `toType`, etc.; this function should not otherwise be
	called directly.
	"""
	assert canCoerce(thing, ty), (thing, ty)

	# If we are in any of the exceptional cases (see the module documentation above),
	# select the appropriate helper function for performing the coercion.
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
		# This is a random value. If we can prove that it will always have the desired
		# type, we return it unchanged; otherwise we wrap it in a TypecheckedDistribution
		# for sample-time typechecking.
		vt = thing._valueType
		if get_type_origin(vt) is typing.Union:
			possibleTypes = get_type_args(vt)
		else:
			possibleTypes = (vt,)
		if all(issubclass(possible, ty) for possible in possibleTypes):
			return thing 	# no coercion necessary
		else:
			return TypecheckedDistribution(thing, realType, error, coercer=coercer)
	elif coercer:
		# The destination type has special coercion rules: call the appropriate helper.
		try:
			return coercer(thing)
		except CoercionFailure as e:
			raise RuntimeParseError(f'{error} ({e.args[0]})') from None
	else:
		# Only instances of the destination type can be coerced into it; since coercion
		# is possible, we must have such an instance, and can return it unchanged.
		return thing

class CoercionFailure(Exception):
	"""Raised by coercion functions when coercion is impossible.

	Only used internally; will be converted to a parse error for reporting to
	the user.
	"""
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
	"""Distribution which typechecks its value at sampling time.

	Only for internal use by the typechecking system; introduced by `coerce` when it is
	unable to guarantee that a random value will have the correct type after sampling.
	Note that the type check is not a purely passive operation, and may actually
	transform the sampled value according to the coercion rules above (e.g. a sampled
	`Point` will be converted to a `Vector` in a context which expects the latter).
	"""

	def __init__(self, dist, ty, errorMessage, coercer=None):
		super().__init__(dist, valueType=ty)
		self._dist = dist
		self._errorMessage = errorMessage
		self._coercer = coercer
		self._loc = saveErrorLocation()

	def sampleGiven(self, value):
		val = value[self._dist]
		suffix = None
		# Attempt to coerce sampled value into the desired type.
		if self._coercer:
			if canCoerceType(type(val), self._valueType):
				try:
					return self._coercer(val)
				except CoercionFailure as e:
					suffix = f' ({e.args[0]})'
		elif isinstance(val, self._valueType):
			return val
		# Coercion failed, so we have a type error.
		if suffix is None:
			suffix = f' (expected {self._valueType.__name__}, got {type(val).__name__})'
		raise RuntimeParseError(self._errorMessage + suffix, self._loc)

	def conditionTo(self, value):
		self._dist.conditionTo(value)

	def __repr__(self):
		return f'TypecheckedDistribution({self._dist}, {self._valueType})'

def coerceToAny(thing, types, error):
	"""Coerce something into any of the given types, raising an error if impossible.

	Only for internal use by the typechecking system; called from `toTypes`.

	Raises:
		RuntimeParseError: if it is impossible to coerce the value into any of the types.
	"""
	for ty in types:
		if canCoerce(thing, ty):
			return coerce(thing, ty, error)
	from scenic.syntax.veneer import verbosePrint
	verbosePrint(f'Failed to coerce {thing} of type {underlyingType(thing)} to {types}',
	             file=sys.stderr)
	raise RuntimeParseError(error)

## Top-level type checking/conversion API

def toTypes(thing, types, typeError='wrong type'):
	"""Convert something to any of the given types, raising an error if impossible.

	Types are tried in the order they are given: the first one compatible with the given
	value is used. Coercions of closely-related types may take place as described in the
	module documentation above.

	If the given value requires lazy evaluation, this function returns a `TypeChecker`
	object that performs the type conversion after specifier resolution.

	Args:
		thing: Value to convert.
		types: Sequence of one or more destination types.
		typeError (str): Message included in exception raised on failure.

	Raises:
		RuntimeParseError: if the given value is not one of the given types and cannot
			be converted to any of them.
	"""
	if needsLazyEvaluation(thing):
		# cannot check the type now; create proxy object to check type after evaluation
		return TypeChecker(thing, types, typeError)
	else:
		return coerceToAny(thing, types, typeError)

def toType(thing, ty, typeError='wrong type'):
	"""Convert something to a given type, raising an error if impossible.

	Equivalent to `toTypes` with a single destination type.
	"""
	return toTypes(thing, (ty,), typeError)

def toScalar(thing, typeError='non-scalar in scalar context'):
	"""Convert something to a scalar, raising an error if impossible.

	See `toTypes` for details.
	"""
	return toType(thing, float, typeError)

def toHeading(thing, typeError='non-heading in heading context'):
	"""Convert something to a heading, raising an error if impossible.

	See `toTypes` for details.
	"""
	return toType(thing, Heading, typeError)

def toVector(thing, typeError='non-vector in vector context'):
	"""Convert something to a vector, raising an error if impossible.

	See `toTypes` for details.
	"""
	return toType(thing, Vector, typeError)

def evaluateRequiringEqualTypes(func, thingA, thingB, typeError='type mismatch'):
	"""Evaluate the func, assuming thingA and thingB have the same type.

	If func produces a lazy value, it should not have any required properties beyond
	those of thingA and thingB.

	Raises:
		RuntimeParseError: if thingA and thingB do not have the same type.
	"""
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
	"""Evaluates a function after checking that two lazy values have the same type."""
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
