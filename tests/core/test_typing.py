
import fractions
from typing import Union, Optional, Tuple, List, Set, FrozenSet

import numpy
import pytest

from scenic.core.distributions import Options
from scenic.core.object_types import Object
from scenic.core.vectors import Vector, OrientedVector, VectorField
from scenic.core.type_support import (canCoerce, canCoerceType, coerce, CoercionFailure,
                                      Heading, get_type_origin, get_type_args)

## Internal utilities

def test_get_type_origin():
    assert get_type_origin(Union[int, set]) is Union
    assert get_type_origin(Optional[int]) is Union
    assert get_type_origin(Tuple[int, set]) is tuple
    assert get_type_origin(List[int]) is list
    assert get_type_origin(Set[float]) is set
    assert get_type_origin(FrozenSet[float]) is frozenset

def test_get_type_args():
    assert get_type_args(Union[int, set]) == (int, set)
    assert get_type_args(Optional[int]) == (int, type(None))
    assert get_type_args(Tuple[int, set]) == (int, set)
    assert get_type_args(List[int]) == (int,)
    assert get_type_args(Set[float]) == (float,)
    assert get_type_args(FrozenSet[float]) == (float,)

## Coercions

def test_coerce_to_scalar():
    good = [42, 3.14, fractions.Fraction(1, 3), numpy.int16(7)]
    assert all(coerce(thing, float) == float(thing) for thing in good)
    assert all(coerce(thing, Heading) == float(thing) for thing in good)

    bad = ['foo', Vector(1, 2), VectorField('', lambda x: 0), [1, 2, 3]]
    assert all(not canCoerce(thing, float) for thing in bad)
    assert all(not canCoerce(thing, Heading) for thing in bad)

def test_coerce_to_heading():
    assert coerce(OrientedVector(1, 2, 0.5), Heading) == pytest.approx(0.5)
    assert coerce(Object(heading=0.42), Heading) == pytest.approx(0.42)

def test_coerce_to_vector():
    def check(thing, answer=None):
        res = coerce(thing, Vector)
        assert isinstance(res, Vector)
        if answer is None:
            answer = thing
        assert res == answer
    check(Vector(1, 2))
    check([1, 2])
    check((1, 2))
    check(Object(position=Vector(4, 9)), answer=(4, 9))
    assert not canCoerce(42, Vector)

def test_coerce_to_class():
    class Dummy: pass
    class SubDummy(Dummy): pass
    d = Dummy()
    s = SubDummy()
    assert coerce(d, Dummy) is d
    assert coerce(s, Dummy) is s
    assert not canCoerce(d, SubDummy)

def test_coerce_union():
    assert canCoerceType(Union[str, int], float)
    assert canCoerce(Options(['foo', 42]), float)
    assert not canCoerce(Options([3.14, 42]), Vector)
