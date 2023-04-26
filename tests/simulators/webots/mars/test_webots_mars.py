
def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('narrowGoal.scenic')
    scenario.generate(maxIterations=1000)
