
from scenic.simulators.newtonian import NewtonianSimulator

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scene, _ = scenario.generate(maxIterations=1000)
    simulator = NewtonianSimulator()
    simulation = simulator.simulate(scene, maxSteps=3)
