import pytest

from scenic.simulators.metadrive import MetaDriveSimulator


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=1)
    simulator = MetaDriveSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    egoPos, otherPos = simulation.result.trajectory[-1]
    assert egoPos.distanceTo(otherPos) < 1
