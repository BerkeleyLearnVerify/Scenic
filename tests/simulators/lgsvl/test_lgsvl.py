
import pytest

from tests.utils import sampleScene, pickle_test, tryPickling

# Suppress potential warning about missing the lgsvl package
pytestmark = pytest.mark.filterwarnings("ignore::scenic.core.simulators.SimulatorInterfaceWarning")

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic', mode_2d=True)
    sampleScene(scenario, maxIterations=1000)

@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic', mode_2d=True)
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled, maxIterations=1000)
    tryPickling(scene)
