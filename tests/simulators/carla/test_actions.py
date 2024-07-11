import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import pytest

try:
    import carla

    from scenic.simulators.carla import CarlaSimulator
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

    for _ in range(30):
        if not isCarlaServerRunning():
            time.sleep(1)

    # extra 3 seconds to ensure server startup
    time.sleep(3)
    return


def terminateCarlaServer():
    result = subprocess.run("lsof -i :2000", capture_output=True, text=True, shell=True)
    if result.stdout:
        lines = result.stdout.strip().split("\n")
        for line in lines[1:]:
            parts = line.split()
            pid = int(parts[1])
            os.kill(pid, signal.SIGTERM)


def isCarlaServerRunning(host="localhost", port=2000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1)  # Set a timeout of 1 second
        try:
            sock.connect((host, port))
            return True
        except (socket.timeout, ConnectionRefusedError):
            return False


@pytest.fixture(scope="package")
def getCarlaSimulator(request):
    if not isCarlaServerRunning():
        launchCarlaServer()

    def cleanup():
        terminateCarlaServer()

    request.addfinalizer(cleanup)

    base = Path(__file__).parent.parent.parent.parent / "assets" / "maps" / "CARLA"

    def _getCarlaSimulator(town):
        path = os.path.join(base, f"{town}.xodr")
        simulator = CarlaSimulator(map_path=path, carla_map=town)
        return simulator, town, path

    return _getCarlaSimulator


def test_throttle(getCarlaSimulator):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = '{town}'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        behavior DriveWithThrottle():
            take SetThrottleAction(1)
        
        ego = new Car at (369, -326), with behavior DriveWithThrottle
        record ego.speed as CarSpeed
        terminate after 50 steps
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

        behavior DriveWithThrottle():
            while True:
                take SetThrottleAction(1)

        behavior Brake():
            while True:
                take SetThrottleAction(0)
                take SetBrakeAction(1)

        behavior DriveThenBrake():
            do DriveWithThrottle() for 25 steps
            do Brake() for 25 steps

        ego = new Car at (369, -326), with behavior DriveThenBrake
        record final ego.speed as CarSpeed
        terminate after 50 steps
    """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    assert records == 0.0
