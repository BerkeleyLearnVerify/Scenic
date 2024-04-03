import re

import pytest

from scenic.simulators.carla import CarlaSimulator
from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def test_atm_object(loadLocalScenario):
    pytest.importorskip("carla")
    scenario = loadLocalScenario("atm.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=10000)
    simulator = CarlaSimulator(
        carla_map="Town05", map_path="../../../assets/maps/CARLA/Town05.xodr"
    )
    simulation = simulator.simulate(scene)
    object_class_type = str(type(simulation.objects[1]))
    atm_class_type = re.search(r"\.([A-Za-z]+)\'>", object_class_type).group(1)
    assert atm_class_type == "ATM"


def test_throttle(loadLocalScenario):
    pytest.importorskip("carla")
    scenario = loadLocalScenario("car_throttle.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=10)
    simulator = CarlaSimulator(
        carla_map="Town05", map_path="../../../assets/maps/CARLA/Town05.xodr"
    )
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    assert records[len(records) // 2][1] < records[-1][1]


def test_brake(loadLocalScenario):
    pytest.importorskip("carla")
    scenario = loadLocalScenario("car_throttle.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=10)
    simulator = CarlaSimulator(
        carla_map="Town05", map_path="../../../assets/maps/CARLA/Town05.xodr"
    )
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    threshold = 3
    assert int(records[-1][1]) < threshold


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
        as action.canBeTakenBy(ego)
    """
    for _ in range(2):
        compileScenic(code, mode2D=True)


@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic", mode2D=True))
    tryPickling(sampleScene(scenario, maxIterations=1000))
