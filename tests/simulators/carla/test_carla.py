import re

import pytest

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def test_all_objects(getAssetPath):
    pytest.importorskip("carla")
    from scenic.simulators.carla import CarlaSimulator

    mapPath = getAssetPath("maps/CARLA/Town05.xodr")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = 'Town05'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        ego = new Car
        atm = new ATM
        advertisement = new Advertisement
        barrel = new Barrel
        barrier = new Barrier
        bench = new Bench
        bicycle = new Bicycle
        box = new Box
        busstop = new BusStop
        casev = new Case
        chair = new Chair
        cone = new Cone
        container = new Container
        creasedbox = new CreasedBox
        debris = new Debris
        garbage = new Garbage
        gnome = new Gnome
        ironplate = new IronPlate
        kiosk = new Kiosk
        mailbox = new Mailbox
        motorcycle = new Motorcycle
        npccar = new NPCCar
        pedestrian = new Pedestrian
        plantpot = new PlantPot
        table = new Table
        trafficwarning = new TrafficWarning
        trash = new Trash
        truck = new Truck
        vendingmachine = new VendingMachine

        terminate after 0.2 seconds
    """
    scenario = compileScenic(code, mode2D=True)
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


def test_throttle(getAssetPath):
    pytest.importorskip("carla")
    from scenic.simulators.carla import CarlaSimulator

    mapPath = getAssetPath("maps/CARLA/Town05.xodr")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = 'Town05'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        behavior DriveWithAppliedThrottle():
            do FollowLaneBehavior() for 2 seconds
            take SetThrottleAction(0.9)

        ego = new Car with behavior DriveWithAppliedThrottle
        record ego.speed as CarSpeed
        terminate after 4 seconds
    """
    scenario = compileScenic(code, mode2D=True)
    scene, _ = scenario.generate(maxIterations=10)
    simulator = CarlaSimulator(
        carla_map="Town05", map_path="../../../assets/maps/CARLA/Town05.xodr"
    )
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    assert records[len(records) // 2][1] < records[-1][1]


def test_brake(getAssetPath):
    pytest.importorskip("carla")
    from scenic.simulators.carla import CarlaSimulator

    mapPath = getAssetPath("maps/CARLA/Town05.xodr")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = 'Town05'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        behavior DriveWithAppliedThrottle():
            do FollowLaneBehavior() for 2 seconds
            take SetBrakeAction(1)

        ego = new Car with behavior DriveWithAppliedThrottle
        record ego.speed as CarSpeed
        terminate after 4 seconds
    """
    scenario = compileScenic(code, mode2D=True)
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
