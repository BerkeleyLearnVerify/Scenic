
from tests.utils import sampleScene, pickle_test, tryPickling

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    sampleScene(scenario, maxIterations=1000)

@pickle_test
def test_pickle(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled, maxIterations=1000)
    tryPickling(scene)
