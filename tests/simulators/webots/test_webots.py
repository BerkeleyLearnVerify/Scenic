import pytest

from tests.utils import pickle_test, sampleScene, tryPickling


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic")
    scenario.generate(maxIterations=1000)


@pickle_test
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic"))
    tryPickling(sampleScene(scenario, maxIterations=1000))
