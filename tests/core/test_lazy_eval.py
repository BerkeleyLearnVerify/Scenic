
from collections import namedtuple

from scenic.core.lazy_eval import DelayedArgument, valueInContext, needsLazyEvaluation

def test_delayed_call():
    da = DelayedArgument(('blob',), lambda context: (lambda x, y=1: x*y + context.blob))
    res = da(42, y=2)
    assert isinstance(res, DelayedArgument)
    Thingy = namedtuple('Thingy', 'blob')
    evres = valueInContext(res, Thingy(blob=-7))
    assert not needsLazyEvaluation(evres)
    assert evres == 77
