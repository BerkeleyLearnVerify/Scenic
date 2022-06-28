
def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scenario.generate(maxIterations=1000)


# def test_simulator(loadLocalScenario):
#     scenario = loadLocalScenario('basic.scenic')
#     scene, _ = scenario.generate(maxIterations=1000)
#     simulator = scenario.getSimulator()
#     simulator.render = False
#     simulator.createSimulation(scene)
