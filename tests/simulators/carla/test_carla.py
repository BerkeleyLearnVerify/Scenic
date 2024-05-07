import os
import subprocess
import time

from flaky import flaky
import pytest

try:
    import carla
except ModuleNotFoundError:
    pytest.skip("carla package not installed", allow_module_level=True)

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def checkCarlaPath():
    CARLA_ROOT = os.environ.get("CARLA_ROOT")
    if not CARLA_ROOT:
        pytest.skip("CARLA_ROOT env variable not set.")
    return CARLA_ROOT


@pytest.fixture
def launchCarlaServer():
    CARLA_ROOT = checkCarlaPath()
    carla_process = subprocess.Popen(
        f"bash {CARLA_ROOT}/CarlaUE4.sh -RenderOffScreen", shell=True
    )
    # NOTE: CARLA server takes time to start up
    time.sleep(3)
    yield
    carla_process.kill()
    time.sleep(5)
    if carla_process.poll() is None:
        carla_process.terminate()
    time.sleep(5)


@pytest.fixture
def getCarlaSimulator(getAssetPath):
    from scenic.simulators.carla import CarlaSimulator

    base = getAssetPath("maps/CARLA")

    def _getCarlaSimulator(town):
        path = os.path.join(base, town + ".xodr")
        simulator = CarlaSimulator(map_path=path, carla_map=town)

        return (simulator, town, path)

    return _getCarlaSimulator


@flaky(max_runs=5, min_passes=1)
def test_throttle(getCarlaSimulator, launchCarlaServer):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = '{town}'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        behavior DriveThenApplyThrottle():
            do FollowLaneBehavior() for 2 seconds
            take SetThrottleAction(0.9)
        
        p = new Point at (369, -326)
        ego = new Car at p, with behavior DriveThenApplyThrottle
        record ego.speed as CarSpeed
        terminate after 4 seconds
    """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    assert records[len(records) // 2][1] < records[-1][1]


@flaky(max_runs=5, min_passes=1)
def test_brake(getCarlaSimulator, launchCarlaServer):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = '{town}'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        behavior DriveThenBrake():
            do FollowLaneBehavior() for 2 seconds
            take SetBrakeAction(1)

        p = new Point at (369, -326)
        ego = new Car at p, with behavior DriveThenBrake
        record ego.speed as CarSpeed
        terminate after 4 seconds
    """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    threshold = 3
    assert int(records[-1][1]) < threshold


def test_map_param_parse(getAssetPath):
    mapPath = getAssetPath("maps/CARLA/Town01.xodr")
    code = f"""
        param map = r'{mapPath}'
        model scenic.simulators.carla.model
        ego = new Car
    """
    scenario = compileScenic(code, mode2D=True)
    assert scenario.params["carla_map"] == "Town01"


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
