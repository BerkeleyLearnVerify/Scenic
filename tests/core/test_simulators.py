import itertools
import random

import pytest

import scenic
from scenic.core.simulators import (
    DummySimulation,
    DummySimulator,
    Simulation,
    SimulatorGroup,
)
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


def test_simulator_bad_scheduler():
    class TestSimulation(DummySimulation):
        def scheduleForAgents(self):
            # Don't include the last agent
            return self.agents[:-1]

    class TestSimulator(DummySimulator):
        def createSimulation(self, scene, **kwargs):
            return TestSimulation(scene, **kwargs)

    scenario = compileScenic(
        """
        behavior Foo():
            take 1

        class TestObj:
            allowCollisions: True
            behavior: Foo

        for _ in range(5):
            new TestObj
        """
    )

    scene, _ = scenario.generate(maxIterations=1)
    simulator = TestSimulator()
    with pytest.raises(RuntimeError):
        result = simulator.simulate(scene, maxSteps=2)


@pytest.mark.slow
def test_simulator_group():
    scenario = compileScenic(
        """
        behavior Foo():
            while True:
                require Range(0,1) < 0.99
                wait

        new Object with behavior Foo()
        """
    )

    for numWorkers, serialized, scene_stream, sim_stream in itertools.product(
        [1, 2], [True, False], [True, False], [True, False]
    ):
        if scene_stream:
            scenes = scenario.generateStream(
                200, numWorkers=numWorkers, serialized=serialized, iterationCount=False
            )
        else:
            scenes, _ = scenario.generateBatch(
                200, numWorkers=numWorkers, serialized=serialized
            )

        sim_group = SimulatorGroup(
            numWorkers=2, simulatorClass=DummySimulator, mute=False
        )

        simulate_params = {"maxSteps": 10}

        if sim_stream:
            results = tuple(
                result
                for _, result in sim_group.simulateStream(
                    scenario,
                    scenes,
                    simulateParams=simulate_params,
                    serialized=serialized,
                )
            )
        else:
            results = sim_group.simulateBatch(
                scenario, scenes, simulateParams=simulate_params, serialized=serialized
            )

        assert any(val is None for val in results)
        assert any(val is not None for val in results)


def test_simulator_group_deterministic():
    scenario = compileScenic(
        """
        behavior Foo():
            while True:
                require Range(0,1) < 0.99
                wait

        new Object with behavior Foo()
        """
    )

    seed = random.getrandbits(32)

    scenic.setSeed(seed)
    scenes, _ = scenario.generateBatch(200, serialized=True)

    sim_group = SimulatorGroup(numWorkers=4, simulatorClass=DummySimulator, mute=False)
    simulate_params = {"maxSteps": 10}

    results1 = tuple(
        result
        for _, result in sim_group.simulateStream(
            scenario, scenes, simulateParams=simulate_params, deterministic=True
        )
    )

    scenic.setSeed(seed)
    scenes, _ = scenario.generateBatch(200, serialized=True)

    sim_group = SimulatorGroup(numWorkers=4, simulatorClass=DummySimulator, mute=False)
    simulate_params = {"maxSteps": 10}

    results2 = tuple(
        result
        for _, result in sim_group.simulateStream(
            scenario, scenes, simulateParams=simulate_params, deterministic=True
        )
    )

    assert len(results1) == len(results2)
    assert all((v1 is None) == (v2 is None) for v1, v2 in zip(results1, results2))
