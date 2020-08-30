
import pytest

from scenic import scenarioFromString as compileScenic

pytest.importorskip('carla')

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scenario.generate(maxIterations=1000)
