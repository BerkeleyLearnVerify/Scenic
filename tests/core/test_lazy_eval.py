
from scenic.core.distributions import Range
from scenic.core.lazy_eval import (LazilyEvaluable, DelayedArgument, valueInContext,
                                   needsLazyEvaluation)
from scenic.core.vectors import Vector

def test_delayed_call():
    da = DelayedArgument(('blob',), lambda context: (lambda x, y=1: x*y + context.blob))
    res = da(42, y=2)
    assert isinstance(res, DelayedArgument)
    context = LazilyEvaluable.makeContext(blob=-7)
    evres = valueInContext(res, context)
    assert not needsLazyEvaluation(evres)
    assert evres == 77

def test_non_lazy_distribution():
    r = Range(0, 10)
    context = LazilyEvaluable.makeContext()
    v = valueInContext(r, context)
    assert v is r

def test_non_lazy_vector():
    r = Range(0, 10)
    vec = Vector(r, 5)
    context = LazilyEvaluable.makeContext()
    v = valueInContext(vec, context)
    assert v[0] is r
