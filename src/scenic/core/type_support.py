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
        * Tuples and lists of length 2 or 3
        * Any type with a **toVector** method (including `Point`)
    * Coercible to a `Behavior`:
        * Subclasses of `Behavior` (coerced by calling them with no arguments)
        * `None` (considered to have type `Behavior` for convenience)
"""

import inspect
import numbers
import sys
import typing
from typing import get_args as get_type_args, get_origin as get_type_origin

from scenic.core.distributions import (
    Distribution,
    RejectionException,
    StarredDistribution,
    TupleDistribution,
    distributionFunction,
    supportInterval,
    toDistribution,
)
from scenic.core.errors import saveErrorLocation
from scenic.core.lazy_eval import (
    DelayedArgument,
    needsLazyEvaluation,
    requiredProperties,
    valueInContext,
)

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
    elif isinstance(thing, TypeChecker):
        return object
    else:
        return type(thing)


def isA(thing, ty):
    """Is this guaranteed to evaluate to a member of the given Scenic type?"""
    # TODO: Remove this hack once type system is smarter.
    if not isinstance(underlyingType(thing), type):
        return False

    return issubclass(underlyingType(thing), ty)


def unifyingType(opts):  # TODO improve?
    """Most specific type unifying the given values."""
    # Gather underlying types of all options
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
    # Compute unifying type
    return unifierOfTypes(types)


def unifierOfTypes(types):
    """Most specific type unifying the given types."""
    # If all types are equal, unifier is trivial
    first = types[0]
    if all(ty == first for ty in types):
        return first
    # Otherwise, erase type parameters which we don't understand
    simpleTypes = []
    for ty in types:
        origin = get_type_origin(ty)
        simpleTypes.append(origin if origin else ty)
    # Scalar types unify to float
    if all(issubclass(ty, numbers.Real) for ty in simpleTypes):
        return float
    # For all other types, find first common ancestor in MRO
    # (ignoring type parameters and skipping ABCs)
    mro = inspect.getmro(simpleTypes[0])
    for ancestor in mro:
        if inspect.isabstract(ancestor):
            continue
        if all(issubclass(ty, ancestor) for ty in simpleTypes):
            return ancestor
    raise AssertionError(f"broken MRO for types {types}")


## Type coercions (for internal use -- see the type checking API below)


def canCoerceType(typeA, typeB):
    """Can values of typeA be coerced into typeB?"""
    originA = get_type_origin(typeA)
    if originA is typing.Union:
        # only raise an error now if none of the possible types will work;
        # we'll do more careful checking at runtime
        return any(canCoerceType(ty, typeB) for ty in get_type_args(typeA))
    elif originA:
        # erase type parameters which we don't know how to use
        typeA = originA
    if typeB == float:
        return issubclass(typeA, numbers.Real)
    elif typeB == Heading:
        from scenic.core.vectors import Orientation

        return (
            canCoerceType(typeA, float)
            or hasattr(typeA, "toHeading")
            or issubclass(typeA, Orientation)
        )
    elif hasattr(typeB, "_canCoerceType"):
        return typeB._canCoerceType(typeA)
    else:
        return issubclass(typeA, typeB)


def canCoerce(thing, ty):
    """Can this value be coerced into the given type?"""
    tt = underlyingType(thing)
    if canCoerceType(tt, ty):
        return True
    elif isinstance(thing, Distribution):
        return True  # fall back on type-checking at runtime
    else:
        return False


def coerce(thing, ty, error="wrong type"):
    """Coerce something into the given type.

    Used internally by `toType`, etc.; this function should not otherwise be
    called directly.
    """
    assert canCoerce(thing, ty), (thing, ty)

    # If we are in any of the exceptional cases (see the module documentation above),
    # select the appropriate helper function for performing the coercion.
    realType = ty
    if ty == float:
        coercer = coerceToFloat
    elif ty == Heading:
        coercer = coerceToHeading
        ty = numbers.Real
        realType = float
    else:
        coercer = getattr(ty, "_coerce", None)

    # Special case: can coerce TupleDistribution directly to Vector.
    # (all other distributions require the usual checking below)
    from scenic.core.vectors import Vector

    if isinstance(thing, TupleDistribution) and ty is Vector:
        length = len(thing)
        if not (2 <= length <= 3):
            msg = f"expected vector, got {thing.builder.__name__} of length {length}"
            raise TypeError(f"{error} ({msg})")
        return Vector(*thing)

    if isinstance(thing, Distribution):
        # This is a random value. If we can prove that it will always have the desired
        # type, we return it unchanged; otherwise we wrap it in a TypecheckedDistribution
        # for sample-time typechecking.
        vt = thing._valueType
        origin = get_type_origin(vt)
        if origin is typing.Union:
            possibleTypes = get_type_args(vt)
        elif origin:
            # Erase type parameters which we don't understand
            possibleTypes = (origin,)
        else:
            possibleTypes = (vt,)
        if all(issubclass(possible, ty) for possible in possibleTypes):
            return thing  # no coercion necessary
        else:
            return TypecheckedDistribution(thing, realType, error, coercer=coercer)
    elif coercer:
        # The destination type has special coercion rules: call the appropriate helper.
        try:
            return coercer(thing)
        except CoercionFailure as e:
            raise TypeError(f"{error} ({e.args[0]})") from None
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


def coerceToHeading(thing) -> Heading:
    from scenic.core.vectors import Orientation

    if isinstance(thing, Orientation):
        return thing.yaw

    h = thing.toHeading() if hasattr(thing, "toHeading") else float(thing)
    return h


class TypecheckedDistribution(Distribution):
    """Distribution which typechecks its value at sampling time.

    Only for internal use by the typechecking system; introduced by `coerce` when it is
    unable to guarantee that a random value will have the correct type after sampling.
    Note that the type check is not a purely passive operation, and may actually
    transform the sampled value according to the coercion rules above (e.g. a sampled
    `Point` will be converted to a `Vector` in a context which expects the latter).
    """

    _deterministic = True

    def __init__(self, dist, ty, errorMessage, coercer=None):
        super().__init__(dist, valueType=ty)
        self._dist = dist
        self._errorMessage = errorMessage
        self._coercer = coercer
        if not coercer:
            # Erase any type parameters, which we don't attempt to check
            origin = get_type_origin(ty)
            self._checkType = origin if origin else ty
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
                    suffix = f" ({e.args[0]})"
        elif isinstance(val, self._checkType):
            return val
        # Coercion failed, so we have a type error.
        if suffix is None:
            suffix = f" (expected {self._valueType.__name__}, got {type(val).__name__})"
        exc = TypeError(self._errorMessage + suffix)
        exc._scenic_location = self._loc
        raise exc

    def conditionTo(self, value):
        self._dist.conditionTo(value)

    def supportInterval(self):
        return supportInterval(self._dist)

    def __repr__(self):
        return f"TypecheckedDistribution({self._dist!r}, {self._valueType!r})"


def coerceToAny(thing, types, error):
    """Coerce something into any of the given types, raising an error if impossible.

    Only for internal use by the typechecking system; called from `toTypes`.

    Raises:
        TypeError: if it is impossible to coerce the value into any of the types.
    """
    for ty in types:
        if canCoerce(thing, ty):
            return coerce(thing, ty, error)
    from scenic.syntax.veneer import verbosePrint

    verbosePrint(
        f"Failed to coerce {thing} of type {underlyingType(thing)} to {types}",
        file=sys.stderr,
    )
    raise TypeError(error)


## Top-level type checking/conversion API


def toTypes(thing, types, typeError="wrong type"):
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
        TypeError: if the given value is not one of the given types and cannot
            be converted to any of them.
    """
    thing = toDistribution(thing)
    if needsLazyEvaluation(thing):
        # cannot check the type now; create proxy object to check type after evaluation
        return TypeChecker(thing, types, typeError)
    else:
        return coerceToAny(thing, types, typeError)


def toType(thing, ty, typeError="wrong type"):
    """Convert something to a given type, raising an error if impossible.

    Equivalent to `toTypes` with a single destination type.
    """
    return toTypes(thing, (ty,), typeError)


def toScalar(thing, typeError="non-scalar in scalar context"):
    """Convert something to a scalar, raising an error if impossible.

    See `toTypes` for details.
    """
    return toType(thing, float, typeError)


def toHeading(thing, typeError="non-heading in heading context"):
    """Convert something to a heading, raising an error if impossible.

    See `toTypes` for details.
    """
    return toType(thing, Heading, typeError)


def toOrientation(thing, typeError="non-orientation in orientation context"):
    """Convert something to an orientation, raising an error if impossible.

    See `toTypes` for details.
    """
    from scenic.core.vectors import Orientation

    return toType(thing, Orientation, typeError)


def toVector(thing, typeError="non-vector in vector context"):
    """Convert something to a vector, raising an error if impossible.

    See `toTypes` for details.
    """
    from scenic.core.vectors import Vector

    return toType(thing, Vector, typeError)


def evaluateRequiringEqualTypes(func, thingA, thingB, typeError="type mismatch"):
    """Evaluate the func, assuming thingA and thingB have the same type.

    If func produces a lazy value, it should not have any required properties beyond
    those of thingA and thingB.

    Raises:
        TypeError: if thingA and thingB do not have the same type.
    """
    if not needsLazyEvaluation(thingA) and not needsLazyEvaluation(thingB):
        if underlyingType(thingA) is not underlyingType(thingB):
            raise TypeError(typeError)
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

    def __repr__(self):
        return f"TypeChecker({self.inner!r},{self.types!r})"


class TypeEqualityChecker(DelayedArgument):
    """Evaluates a function after checking that two lazy values have the same type."""

    def __init__(self, func, checkA, checkB, error):
        props = requiredProperties(checkA) | requiredProperties(checkB)

        def check(context):
            ca = valueInContext(checkA, context)
            cb = valueInContext(checkB, context)
            if underlyingType(ca) is not underlyingType(cb):
                raise TypeError(error)
            return valueInContext(func(), context)

        super().__init__(props, check)
        self.inner = func
        self.checkA = checkA
        self.checkB = checkB

    def __repr__(self):
        return f"TypeEqualityChecker({self.inner!r},{self.checkA!r},{self.checkB!r})"


## Utilities


def is_typing_generic(tp):
    """Whether this is a pre-3.9 generic type from the typing module."""
    return isinstance(tp, typing._GenericAlias)
