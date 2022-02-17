from scenic.core.lazy_eval import (
    LazilyEvaluable,
    DelayedArgument,
    valueInContext,
    needsLazyEvaluation,
)


def test_delayed_call():
    da = DelayedArgument(
        ("blob",), lambda context: (lambda x, y=1: x * y + context.blob)
    )
    res = da(42, y=2)
    assert isinstance(res, DelayedArgument)
    context = LazilyEvaluable.makeContext(blob=-7)
    evres = valueInContext(res, context)
    assert not needsLazyEvaluation(evres)
    assert evres == 77
