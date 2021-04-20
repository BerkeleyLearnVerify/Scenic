
from scenic.core.vectors import *
from scenic.core.lazy_eval import (LazilyEvaluable, DelayedArgument, valueInContext,
                                   needsLazyEvaluation)
from scenic.core.distributions import Options, underlyingFunction

def test_equality():
    v = Vector(1, 4)
    assert v == Vector(1, 4)
    assert v != Vector(1, 5)
    assert v == (1, 4)
    assert v != (1,)
    assert v != (1, 4, 9)
    assert v == [1, 4]
    assert v != [1]
    assert v != [1, 4, 9]

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
