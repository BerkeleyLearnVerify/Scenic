
import pytest

# TODO make this work with plain old pickle, rather than dill?
# (currently pickle chokes on various decorators and local functions)
pytest.importorskip('dill')
import dill

from scenic.core.distributions import Options, Range, Normal, TruncatedNormal, distributionMethod
from scenic.core.geometry import hypot
from scenic.core.object_types import Object
from scenic.core.specifiers import Specifier
from scenic.core.lazy_eval import DelayedArgument
from scenic.core.utils import areEquivalent
from tests.utils import compileScenic

def tryPickling(thing, checkEquivalence=True, pickler=dill):
    pickled = pickler.dumps(thing)
    unpickled = pickler.loads(pickled)
    if checkEquivalence:
        assert areEquivalent(unpickled, thing)

def test_pickle_distribution_basic():
    dist = Options({
        Range(0, 1): 2,
        Options([12 + Range(-1, 3), Normal(4, 5)]): 1,
        (3, TruncatedNormal(1, 2, 3, 4)): 1,
    })
    tryPickling(dist)

def test_pickle_distribution_function():
    # N.B. cloudpickle fails this test; it seems to not restore the __globals__ of the
    # hypot wrapper properly.
    dist = hypot(Range(0, 1), 2).hex()
    tryPickling(dist)

@pytest.mark.xfail(reason='dill cannot yet pickle this (issue #332)', strict=True)
def test_pickle_distribution_method():
    # N.B. cloudpickle *does* work in this case, but it's less common than the one above
    class Foo:
        def __init__(self, blob):
            self.blob = blob

        @distributionMethod
        def bar(self, x):
            return x if x > 0 else -x

        def __eq__(self, other):
            return type(other) is Foo and self.blob == other.blob

    dist = Foo(5).bar(Range(0, 1))
    tryPickling(dist)

def test_pickle_object():
    spec = Specifier('blob',
                     DelayedArgument(('width',), lambda context: 2 * context.width),
                     {'width'})
    obj = Object(spec)
    tryPickling(obj)

def test_pickle_scenario():
    scenario = compileScenic('ego = Object with width Range(1, 2)')
    tryPickling(scenario)
