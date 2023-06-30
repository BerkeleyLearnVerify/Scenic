import pytest

from scenic.core.distributions import (
    Normal,
    Options,
    Range,
    TruncatedNormal,
    distributionMethod,
)
from scenic.core.geometry import hypot
from scenic.core.lazy_eval import DelayedArgument
from scenic.core.object_types import Object
from scenic.core.simulators import DummySimulator
from scenic.core.specifiers import Specifier
from tests.utils import (
    compileScenic,
    pickle_test,
    sampleEgo,
    sampleScene,
    sampleSceneFrom,
    tryPickling,
)

pytestmark = pickle_test


def test_pickle_distribution_basic():
    dist = Options(
        {
            Range(0, 1): 2,
            Options([12 + Range(-1, 3), Normal(4, 5)]): 1,
            (3, TruncatedNormal(1, 2, 3, 4)): 1,
        }
    )
    tryPickling(dist)


def test_pickle_distribution_function():
    # N.B. cloudpickle fails this test; it seems to not restore the __globals__ of the
    # hypot wrapper properly.
    dist = hypot(Range(0, 1), 2).hex()
    tryPickling(dist)


def test_pickle_distribution_method():
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
    spec = Specifier(
        "unnamed",
        {"blob": 1},
        DelayedArgument(("width",), lambda context: {"blob": 2 * context.width}),
    )
    obj = Object._withSpecifiers((spec,))
    tryPickling(obj)


def test_pickle_scenario():
    scenario = compileScenic(
        """
        ego = new Object at (Range(1, 2), 0)
        other = new Object at (Range(1, 2), 5)
        require ego.x < other.x
    """
    )
    sc1 = tryPickling(scenario)
    sc2 = tryPickling(scenario)
    s1 = sampleScene(sc1, maxIterations=60)
    s2 = sampleScene(sc2, maxIterations=60)
    e1, e2 = s1.egoObject, s2.egoObject
    o1, o2 = s1.objects[1], s2.objects[1]
    assert 1 <= e1.x <= 2
    assert 1 <= e2.x <= 2
    assert e1.x != e2.x
    assert e1.x < o1.x
    assert e2.x < o2.x
    assert o1.x != o2.x


def test_pickle_scene():
    scene = sampleSceneFrom("ego = new Object at Range(1, 2) @ 3")
    tryPickling(scene)


def test_pickle_scenario_dynamic():
    scenario = compileScenic(
        """
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
    """
    )
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled)
    sim = DummySimulator()
    sim.simulate(scene, maxSteps=2, maxIterations=1)


def test_pickle_scenario_dynamic_default_global():
    sc1 = compileScenic(
        """
        def helper():
            return 42
        class Thing:
            blah[dynamic]: helper()
        ego = new Thing
    """
    )
    sc2 = tryPickling(sc1)
    ego = sampleEgo(sc2)
    assert ego.blah == 42
