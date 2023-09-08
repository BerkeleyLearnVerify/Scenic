import pytest

from scenic.simulators.newtonian import NewtonianSimulator
from tests.utils import pickle_test, sampleScene, tryPickling


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic")
    scene, _ = scenario.generate(maxIterations=1)
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    egoPos, otherPos = simulation.result.trajectory[-1]
    assert egoPos.distanceTo(otherPos) < 1


@pytest.mark.graphical
def test_render(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic")
    scene, _ = scenario.generate(maxIterations=1)
    simulator = NewtonianSimulator(render=True)
    simulator.simulate(scene, maxSteps=3)


def test_driving(loadLocalScenario):
    def check():
        scenario = loadLocalScenario("driving.scenic", mode2D=True)
        scene, _ = scenario.generate(maxIterations=1000)
        simulator = scenario.getSimulator()
        simulation = simulator.simulate(scene, maxSteps=3)

    # Run this twice to catch leaks between successive compilations.
    check()
    check()  # If we fail here, something is leaking.


@pickle_test
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic"))
    scene = tryPickling(sampleScene(scenario, maxIterations=1))
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    egoPos, otherPos = simulation.result.trajectory[-1]
    assert egoPos.distanceTo(otherPos) < 1
