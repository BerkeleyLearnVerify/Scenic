
import pytest

# TODO make this work with plain old pickle, rather than dill?
# (currently pickle chokes on various decorators and local functions)
pytest.importorskip('dill')
import dill

from scenic.core.distributions import Options, Range
from scenic.core.object_types import Object
from scenic.core.specifiers import Specifier
from scenic.core.lazy_eval import DelayedArgument
from scenic.core.utils import areEquivalent
from tests.utils import compileScenic

def tryPickling(thing, checkEquivalence=True):
    pickled = dill.dumps(thing)
    unpickled = dill.loads(pickled)
    if checkEquivalence:
        assert areEquivalent(unpickled, thing)

def test_pickle_distribution():
    dist = Options({
        Range(0, 1): 2,
        Options([12 + Range(-1, 3), Range(4, 5)]): 1
    })
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
