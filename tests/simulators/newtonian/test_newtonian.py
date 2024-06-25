import os
from pathlib import Path

from PIL import Image as IPImage
import pytest

from scenic.domains.driving.roads import Network
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
    simulator = NewtonianSimulator()
    simulator.simulate(scene, maxSteps=3)


def test_driving_2D(loadLocalScenario):
    def check():
        scenario = loadLocalScenario("driving.scenic", mode2D=True)
        scene, _ = scenario.generate(maxIterations=1000)
        simulator = scenario.getSimulator()
        simulation = simulator.simulate(scene, maxSteps=3)

    # Run this twice to catch leaks between successive compilations.
    check()
    check()  # If we fail here, something is leaking.


@pytest.mark.graphical
def test_gif_creation(loadLocalScenario):
    scenario = loadLocalScenario("driving.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=1000)
    path = Path("assets") / "maps" / "CARLA" / "Town01.xodr"
    network = Network.fromFile(path)
    simulator = NewtonianSimulator(network=network, export_gif=True)
    simulation = simulator.simulate(scene, maxSteps=100)
    gif_path = Path("") / "simulation.gif"
    assert os.path.exists(gif_path)


@pickle_test
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic"))
    scene = tryPickling(sampleScene(scenario, maxIterations=1))
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    egoPos, otherPos = simulation.result.trajectory[-1]
    assert egoPos.distanceTo(otherPos) < 1
