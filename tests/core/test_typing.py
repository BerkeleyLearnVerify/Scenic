from contextlib import nullcontext
import fractions
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple, Union

import numpy
import pytest

from scenic.core.distributions import DiscreteRange, Options, Range, distributionFunction
from scenic.core.object_types import Object
from scenic.core.type_support import (
    CoercionFailure,
    Heading,
    TypecheckedDistribution,
    canCoerce,
    canCoerceType,
    coerce,
    get_type_args,
    get_type_origin,
    underlyingType,
    unifyingType,
)
from scenic.core.vectors import Orientation, OrientedVector, Vector, VectorField

## Helpers


def makeDistWithType(ty):
    @distributionFunction
    def helper(x) -> ty:
        assert False  # should never get here

    return helper(Range(0, 1))


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

    bad = ["foo", Vector(1, 2), VectorField("", lambda x: 0), [1, 2, 3, 4]]
    assert all(not canCoerce(thing, float) for thing in bad)
    assert all(not canCoerce(thing, Heading) for thing in bad)


def test_coerce_to_heading():
    assert coerce(OrientedVector(1, 2, 0, 0.5), Heading) == pytest.approx(0.5)
    assert coerce(Object._with(yaw=0.42), Heading) == pytest.approx(0.42)


def test_coerce_to_vector():
    def check(thing, answer=None):
        res = coerce(thing, Vector)
        assert isinstance(res, Vector)
        if answer is None:
            answer = thing
        assert res == answer

    check([1, 2])
    check((1, 2))
    check(Vector(1, 2))
    check(Vector(1, 2, 0))
    check([1, 2, 0])
    check((1, 2, 0))
    check(Object._with(position=Vector(4, 9)), answer=(4, 9, 0))
    check(Object._with(position=Vector(4, 9, 0)), answer=(4, 9, 0))
    assert not canCoerce(42, Vector)
    with pytest.raises(TypeError):
        coerce([1, 2, 3, 4], Vector)


def test_coerce_to_class():
    class Dummy:
        pass

    class SubDummy(Dummy):
        pass

    d = Dummy()
    s = SubDummy()
    assert coerce(d, Dummy) is d
    assert coerce(s, Dummy) is s
    assert not canCoerce(d, SubDummy)


def test_coerce_union():
    assert canCoerceType(Union[str, int], float)
    assert canCoerce(Options(["foo", 42]), float)


def test_coerce_parameters():
    assert canCoerceType(List[int], list)
    assert canCoerceType(Tuple[int, str], tuple)
    assert canCoerceType(Tuple[int, ...], tuple)
    assert canCoerceType(Dict[str, int], dict)
    assert not canCoerceType(List[float], float)


def test_coerce_distribution_scalar():
    x = Options([1, 4, 9])
    assert coerce(x, int) is x
    x = Options([3.14, 2.718])
    assert coerce(x, float) is x
    x = Options([3.14, "zoggle"])
    y = coerce(x, float)
    assert isinstance(y, TypecheckedDistribution)
    with pytest.raises(TypeError):
        for _ in range(60):
            y.sample()


def test_coerce_distribution_vector():
    x = Options([Vector(1, 2), Vector(3, 4)])
    assert coerce(x, Vector) is x

    def check(values, fail=False):
        x = Options(values)
        y = coerce(x, Vector)
        assert isinstance(y, TypecheckedDistribution)
        manager = pytest.raises(TypeError) if fail else nullcontext()
        with manager:
            for _ in range(60):
                y.sample()

    check([Vector(1, 2), (3, 4)])
    check([Vector(1, 2), 42], fail=True)
    check([(1, 2), (3, 4)])
    check([(1, 2), (1, 2, 3, 4)], fail=True)


def test_coerce_distribution_tuple():
    y = makeDistWithType(Tuple[float, str])
    assert coerce(y, tuple) is y


## Type inference


def test_unifying_type():
    assert unifyingType([1, 4, 9]) is int
    assert unifyingType([3.14, 42]) is float
    assert unifyingType([1, "foo"]) is object
    assert unifyingType([(1, 2), (3, 4)]) is tuple

    class Dummy:
        pass

    class SubDummy(Dummy):
        pass

    assert unifyingType([SubDummy(), SubDummy()]) is SubDummy
    assert unifyingType([Dummy(), SubDummy()]) is Dummy
    assert unifyingType([SubDummy(), Dummy()]) is Dummy

    y = makeDistWithType(List[int])
    assert unifyingType([y, y]) == List[int]
    assert unifyingType([y, ["foo"]]) is list


def test_infer_scalar_operator():
    x = Range(0, 1)
    y = x + x
    assert underlyingType(y) is float


def test_infer_getitem_list():
    y = makeDistWithType(List[float])
    assert underlyingType(y[2]) is float
    assert underlyingType(y[1:3]) == List[float]
    i = DiscreteRange(0, 1)
    assert underlyingType(y[i]) is float
    assert underlyingType(y[i : i + 2]) == List[float]


def test_infer_getitem_tuple_fixed():
    y = makeDistWithType(Tuple[float, str])
    assert underlyingType(y[0]) is float
    assert underlyingType(y[1]) is str
    assert underlyingType(y[:1]) == Tuple[float]
    i = DiscreteRange(0, 1)
    assert underlyingType(y[i]) == Union[float, str]
    assert underlyingType(y[i:i]) == Tuple[Union[float, str], ...]


def test_infer_getitem_tuple_fixed_any():
    y = makeDistWithType(Tuple[float, Any])
    assert underlyingType(y[1]) is object
    assert underlyingType(y[1:]) == Tuple[Any]
    i = DiscreteRange(0, 1)
    assert underlyingType(y[i]) is object
    assert underlyingType(y[i:i]) == Tuple


def test_infer_getitem_tuple_variable():
    y = makeDistWithType(Tuple[str, ...])
    assert underlyingType(y[0]) is str
    assert underlyingType(y[:1]) == Tuple[str, ...]
    i = DiscreteRange(0, 1)
    assert underlyingType(y[i]) is str
    assert underlyingType(y[i : i + 1]) == Tuple[str, ...]


def test_infer_getitem_dict():
    y = makeDistWithType(Dict[int, float])
    assert underlyingType(y[0]) is float
    i = DiscreteRange(0, 1)
    assert underlyingType(y[i]) is float


def test_infer_getitem_str():
    y = Options(["phobos", "deimos"])
    assert underlyingType(y[0]) is str
    assert underlyingType(y[1:3]) is str
    i = DiscreteRange(0, 1)
    assert underlyingType(y[i]) is str
    assert underlyingType(y[i : i + 1]) is str


def test_infer_union_operator():
    x = makeDistWithType(Union[float, int])
    y = x + x
    assert underlyingType(y) is float
    x = makeDistWithType(Union[float, str])
    y = x + x
    assert underlyingType(y) is object
    x = makeDistWithType(Union[List[int], Tuple[int, ...]])
    y = x[2]
    assert underlyingType(y) is int


def test_infer_optional_operator():
    x = makeDistWithType(Optional[float])
    y = x + x
    assert underlyingType(y) is float
    x = makeDistWithType(Optional[List[int]])
    y = x[2]
    assert underlyingType(y) is int


def test_infer_union_attribute():
    class Foo:
        a: int

    class Bar:
        a: int

    x = makeDistWithType(Union[Foo, Bar])
    assert underlyingType(x.a) is int

    class Baz:
        a: str

    x = makeDistWithType(Union[Foo, Baz])
    assert underlyingType(x.a) is object
    x = makeDistWithType(Union[Foo, str])
    assert underlyingType(x.a) is object


def test_infer_optional_attribute():
    class Foo:
        a: str

    x = makeDistWithType(Optional[Foo])
    assert underlyingType(x.a) is str
