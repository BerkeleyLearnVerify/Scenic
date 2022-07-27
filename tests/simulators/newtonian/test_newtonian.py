
from scenic.simulators.newtonian import NewtonianSimulator

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scene, _ = scenario.generate(maxIterations=1)
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=100)
    egoPos, otherPos = simulation.result.trajectory[-1]
    assert egoPos.distanceTo(otherPos) < 1

def test_driving(loadLocalScenario):
    scenario = loadLocalScenario('driving.scenic')
    scene, _ = scenario.generate(maxIterations=1000)
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=3)
