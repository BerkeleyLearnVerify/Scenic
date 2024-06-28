import os
import socket
import subprocess

import pytest

try:
    import carla
except ModuleNotFoundError:
    pytest.skip("carla package not installed", allow_module_level=True)

from tests.utils import compileScenic, sampleScene


def checkCarlaPath():
    CARLA_ROOT = os.environ.get("CARLA_ROOT")
    if not CARLA_ROOT:
        pytest.skip("CARLA_ROOT env variable not set.")
    return CARLA_ROOT


def launchCarlaServer():
    CARLA_ROOT = checkCarlaPath()
    subprocess.Popen(f"bash {CARLA_ROOT}/CarlaUE4.sh -RenderOffScreen", shell=True)


def isCarlaServerRunning(host="localhost", port=2000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1)  # Set a timeout of 1 second
        try:
            sock.connect((host, port))
            return True
        except (socket.timeout, ConnectionRefusedError):
            return False


@pytest.fixture
def getCarlaSimulator(getAssetPath):
    from scenic.simulators.carla import CarlaSimulator

    base = getAssetPath("maps/CARLA")

    def _getCarlaSimulator(town):
        if not isCarlaServerRunning():
            launchCarlaServer()
        path = os.path.join(base, town + ".xodr")
        simulator = CarlaSimulator(map_path=path, carla_map=town)

        return (simulator, town, path)

    return _getCarlaSimulator


def test_throttle(getCarlaSimulator):
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


def test_brake(getCarlaSimulator):
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
