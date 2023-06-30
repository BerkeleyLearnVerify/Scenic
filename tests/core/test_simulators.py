import pytest

from scenic.core.simulators import DummySimulation, DummySimulator, Simulation
from tests.utils import compileScenic, sampleResultFromScene, sampleSceneFrom


def test_old_style_simulator():
    with pytest.raises(RuntimeError) as e:

        class MySimulation(Simulation):
            def run(*args, **kwargs):
                assert False

    assert "old-style simulation API" in str(e)

    with pytest.raises(RuntimeError) as e:

        class MySimulation(Simulation):
            def createObject(*args, **kwargs):
                assert False

    assert "old-style simulation API" in str(e)


def test_simulator_destruction():
    sim = DummySimulator()
    scene = sampleSceneFrom("ego = new Object")
    sim.simulate(scene, maxSteps=1)
    sim.destroy()
    with pytest.raises(RuntimeError) as e:
        sim.simulate(scene, maxSteps=1)
    assert "simulator cannot run additional simulations" in str(e)
    with pytest.raises(RuntimeError) as e:
        sim.destroy()
    assert "destroy() called twice" in str(e)


def test_simulator_set_property():
    class TestSimulation(DummySimulation):
        def createObjectInSimulator(self, obj):
            super().createObjectInSimulator(obj)
            obj.foo = "bar"

    class TestSimulator(DummySimulator):
        def createSimulation(self, scene, **kwargs):
            return TestSimulation(scene, drift=self.drift, **kwargs)

    scenario = compileScenic(
        """
        class TestObj:
            foo: None

        ego = new TestObj

        record ego.foo as test_val_1
        record initial ego.foo as test_val_2
        record final ego.foo as test_val_3
    """
    )

    scene, _ = scenario.generate(maxIterations=1)
    simulator = TestSimulator()
    result = simulator.simulate(scene, maxSteps=2)
    assert result is not None
    assert result.records["test_val_1"] == [(0, "bar"), (1, "bar"), (2, "bar")]
    assert result.records["test_val_2"] == result.records["test_val_3"] == "bar"
