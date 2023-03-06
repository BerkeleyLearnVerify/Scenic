
import matplotlib.pyplot as plt
import pytest

from scenic.simulators.gta.interface import GTA
from tests.utils import sampleScene, pickle_test, tryPickling

# Skip tests if Pillow or OpenCV not installed
pytest.importorskip("PIL")
pytest.importorskip("cv2")

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scene = sampleScene(scenario, maxIterations=1000)
    GTA.Config(scene)
    scene.show(block=False)
    plt.close()

def test_bumper_to_bumper(loadLocalScenario):
    scenario = loadLocalScenario('bumperToBumper.scenic')
    scene = sampleScene(scenario, maxIterations=1000)
    GTA.Config(scene)
    scene.show(zoom=1, block=False)
    plt.close()

@pickle_test
def test_pickle(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled, maxIterations=1000)
    tryPickling(scene)
