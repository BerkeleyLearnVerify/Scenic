
import pytest

# Skip tests if pyproj not installed
pytest.importorskip("pyproj")

def test_basic(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scenario.generate(maxIterations=1000)

def test_turning_car(loadLocalScenario):
    scenario = loadLocalScenario('turningCar.scenic')
    scenario.generate(maxIterations=1000)
