import pytest

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling
from scenic.simulators.carla import CarlaSimulator

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)

def test_dynamic_throttle(loadLocalScenario):
    scenario = loadLocalScenario("car_throttle.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=1000)
    simulator = CarlaSimulator(carla_map='Town05', map_path="../../../assets/maps/CARLA/Town05.xodr")
    # simulator = scenario.getSimulator()
    result = simulator.simulate(scene, maxSteps=1)
    assert result is not None


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic", mode2D=True)
    scenario.generate(maxIterations=1000)


def test_simulator_import():
    pytest.importorskip("carla")
    from scenic.simulators.carla import CarlaSimulator


def test_consistent_object_type(getAssetPath):
    pytest.importorskip("carla")
    mapPath = getAssetPath("maps/CARLA/Town01.xodr")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = 'Town01'
        model scenic.simulators.carla.model
        action = SetGearAction(0)
        ego = new Car
        assert action.canBeTakenBy(ego)
    """
    for _ in range(2):
        compileScenic(code, mode2D=True)


@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic", mode2D=True))
    tryPickling(sampleScene(scenario, maxIterations=1000))
