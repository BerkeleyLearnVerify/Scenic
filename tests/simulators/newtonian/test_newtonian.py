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
    simulator = NewtonianSimulator(render=True)
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
    simulator = NewtonianSimulator(render=True, network=network, export_gif=True)
    simulation = simulator.simulate(scene, maxSteps=100)
    gif_path = Path("") / "simulation.gif"
    assert os.path.exists(gif_path)
    os.remove(gif_path)


@pickle_test
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic"))
    scene = tryPickling(sampleScene(scenario, maxIterations=1))
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    egoPos, otherPos = simulation.result.trajectory[-1]
    assert egoPos.distanceTo(otherPos) < 1


def test_pedestrian_movement(loadLocalScenario):
    scenario = loadLocalScenario("pedestrian.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=1)
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=8)
    init = simulation.result.records["InitialPos"]
    fin = simulation.result.records["FinalPos"]
    assert init != fin, "Pedestrian did not move."


def test_pedestrian_velocity_vector(loadLocalScenario):
    scenario = loadLocalScenario("pedestrian_velocity.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=1)
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=8)
    init = simulation.result.records["InitialPos"]
    fin = simulation.result.records["FinalPos"]
    dx = fin[0] - init[0]
    dy = fin[1] - init[1]
    # Expect movement northeast (positive dx and dy)
    assert dx > 0.1, f"Expected positive x movement (east), got dx = {dx}"
    assert dy > 0.1, f"Expected positive y movement (north), got dy = {dy}"
