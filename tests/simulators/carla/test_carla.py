
import pytest

from tests.utils import pickle_test, tryPickling, sampleScene

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings("ignore::scenic.core.simulators.SimulatorInterfaceWarning")

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic', mode_2d=True)
    scenario.generate(maxIterations=1000)

@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario('basic.scenic', mode_2d=True))
    tryPickling(sampleScene(scenario, maxIterations=1000))
