import pytest
from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)

def test_throttle(getCarlaSimulator, launchCarlaServer):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator('Town01')
    code = f"""
        param map = r'{mapPath}'
        param carla_map = '{town}'
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
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    assert records[len(records) // 2][1] < records[-1][1]


def test_brake(getCarlaSimulator, launchCarlaServer):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator('Town01')
    code = f"""
        param map = r'{mapPath}'
        param carla_map = '{town}'
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
