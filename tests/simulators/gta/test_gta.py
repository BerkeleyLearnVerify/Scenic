
import pytest

from tests.utils import sampleScene, pickle_test, tryPickling

# Skip tests if Pillow or OpenCV not installed
pytest.importorskip("PIL")
pytest.importorskip("cv2")

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scenario.generate(maxIterations=1000)

def test_bumper_to_bumper(loadLocalScenario):
    scenario = loadLocalScenario('bumperToBumper.scenic')
    scenario.generate(maxIterations=1000)

@pickle_test
def test_pickle(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled, maxIterations=1000)
    tryPickling(scene)
