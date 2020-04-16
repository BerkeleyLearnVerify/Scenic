
from scenic import scenarioFromString as compileScenic

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.sc')
    scenario.generate(maxIterations=1000)
