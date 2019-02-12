
def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('narrowGoal.sc')
    scenario.generate(maxIterations=1000)
