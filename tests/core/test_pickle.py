
import pytest

from scenic.core.distributions import Options, Range, Normal, TruncatedNormal, distributionMethod
from scenic.core.geometry import hypot
from scenic.core.object_types import Object
from scenic.core.specifiers import Specifier
from scenic.core.lazy_eval import DelayedArgument
from scenic.core.simulators import DummySimulator
from tests.utils import (pickle_test, tryPickling,
                         compileScenic, sampleEgo, sampleSceneFrom, sampleScene)

pytestmark = pickle_test

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
    scenario = compileScenic('ego = new Object at Range(1, 2) @ 0')
    sc1 = tryPickling(scenario)
    sc2 = tryPickling(scenario)
    e1, e2 = sampleEgo(sc1), sampleEgo(sc2)
    assert 1 <= e1.position.x <= 2
    assert 1 <= e2.position.x <= 2
    assert e1.position.x != e2.position.x

def test_pickle_scene():
    scene = sampleSceneFrom('ego = new Object at Range(1, 2) @ 3')
    tryPickling(scene)

def test_pickle_scenario_dynamic():
    scenario = compileScenic("""
        model scenic.simulators.newtonian.model
        behavior Foo(x):
            while True:
                wait
        behavior Bar(t):
            do Foo(Range(0, 1)) for t seconds
        ego = new Object with behavior Bar(Range(5, 10))
        other = new Object at 10 @ Range(20, 30)
        require always other.position.x >= 0
        terminate when ego.position.x > 10
        record ego.position as egoPos
    """)
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled)
    sim = DummySimulator(timestep=1)
    sim.simulate(scene, maxSteps=2, maxIterations=1)
