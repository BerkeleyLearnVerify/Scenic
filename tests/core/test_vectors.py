from scenic.core.distributions import Options, underlyingFunction
from scenic.core.lazy_eval import (
    DelayedArgument,
    LazilyEvaluable,
    needsLazyEvaluation,
    valueInContext,
)
from scenic.core.vectors import *


def test_vector_equality():
    v = Vector(1, 4)
    assert v == Vector(1, 4)
    assert v == Vector(1, 4, 0)
    assert v != Vector(1, 5)
    assert v != Vector(1, 4, 9)
    assert v == (1, 4, 0)
    assert v != (1,)
    assert v != (1, 4, 9)
    assert v == [1, 4, 0]
    assert v != [1]
    assert v != [1, 4, 9]
    assert v == (1, 4)


def test_orientation_equality():
    o1 = Orientation.fromEuler(math.pi, math.pi, math.pi)
    o2 = Orientation.fromEuler(-math.pi, -math.pi, -math.pi)
    o3 = Orientation.fromEuler(math.pi, math.pi, 0)

    assert o1 == o1 and o1.approxEq(o1)
    assert o1.approxEq(o2)
    assert o1 != o3 and not o1.approxEq(o3)


def test_distribution_method_encapsulation():
    vf = VectorField("Foo", lambda pos: 0)
    pt = vf.followFrom(Vector(0, 0), Options([1, 2]), steps=1)
    assert isinstance(pt, VectorMethodDistribution)
    assert pt.method is underlyingFunction(vf.followFrom)


def test_distribution_method_encapsulation_lazy():
    vf = VectorField("Foo", lambda pos: 0)
    da = DelayedArgument(set(), lambda context: Options([1, 2]))
    pt = vf.followFrom(Vector(0, 0), da, steps=1)
    assert isinstance(pt, DelayedArgument)
    context = LazilyEvaluable.makeContext()
    evpt = valueInContext(pt, context)
    assert not needsLazyEvaluation(evpt)
    assert isinstance(evpt, VectorMethodDistribution)
    assert evpt.method is underlyingFunction(vf.followFrom)
