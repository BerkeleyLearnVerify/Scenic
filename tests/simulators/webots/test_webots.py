import pytest

from tests.utils import pickle_test, tryPickling, sampleScene


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic")
    scenario.generate(maxIterations=1000)


@pytest.mark.skip(reason="dill cannot pickle ABCs yet")
@pickle_test
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic"))
    tryPickling(sampleScene(scenario, maxIterations=1000))
