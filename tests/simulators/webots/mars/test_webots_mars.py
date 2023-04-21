
def test_basic(loadLocalScenario2D):
    scenario = loadLocalScenario2D('narrowGoal.scenic')
    scenario.generate(maxIterations=1000)
