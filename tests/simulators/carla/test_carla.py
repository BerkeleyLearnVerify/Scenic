import re

import pytest

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def test_all_objects(loadLocalScenario):
    pytest.importorskip("carla")
    from scenic.simulators.carla import CarlaSimulator

    scenario = loadLocalScenario("all_objects.scenic", mode2D=True)
    scene, _ = scenario.generate(maxIterations=10000)
    simulator = CarlaSimulator(
        carla_map="Town05", map_path="../../../assets/maps/CARLA/Town05.xodr"
    )
    simulation = simulator.simulate(scene)
    objects = simulation.objects
    expected_objects = [
        "ATM",
        "Advertisement",
        "Barrel",
        "Barrier",
        "Bench",
        "Bicycle",
        "Box",
        "BusStop",
        "Car",
        "Case",
        "Chair",
        "Cone",
        "Container",
        "CreasedBox",
        "Debris",
        "Garbage",
        "Gnome",
        "IronPlate",
        "Kiosk",
        "Mailbox",
        "Motorcycle",
        "NPCCar",
        "Pedestrian",
        "PlantPot",
        "Table",
        "TrafficWarning",
        "Trash",
        "Truck",
        "VendingMachine",
    ]
    assert len(objects) == len(expected_objects)
    for i in range(len(expected_objects)):
        object = str(type(objects[i]))
        object_class = re.search(r"\.([A-Za-z]+)\'>", object).group(1)
        assert object_class in expected_objects


def test_throttle(loadLocalScenario):
    pytest.importorskip("carla")
    from scenic.simulators.carla import CarlaSimulator

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
    from scenic.simulators.carla import CarlaSimulator

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
        assert action.canBeTakenBy(ego)
    """
    for _ in range(2):
        compileScenic(code, mode2D=True)


@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic", mode2D=True))
    tryPickling(sampleScene(scenario, maxIterations=1000))
