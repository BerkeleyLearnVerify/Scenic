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


def isCarlaServerRunning(host="localhost", port=2000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(1)
        try:
            sock.connect((host, port))
            return True
        except (socket.timeout, ConnectionRefusedError):
            return False


@pytest.fixture(scope="package")
def getCarlaSimulator(getAssetPath):
    carla_process = None
    start_time = time.time()
    if not isCarlaServerRunning():
        CARLA_ROOT = checkCarlaPath()
        carla_process = subprocess.Popen(
            f"bash {CARLA_ROOT}/CarlaUE4.sh -RenderOffScreen", shell=True
        )

        for _ in range(600):
            if isCarlaServerRunning():
                break
            time.sleep(1)
        else:
            pytest.fail("Unable to connect to CARLA.")

        # Log the time it took for CARLA to start
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"CARLA started successfully in {elapsed_time:.2f} seconds.")

        # Extra 5 seconds to ensure server startup
        time.sleep(10)

    base = getAssetPath("maps/CARLA")

    def _getCarlaSimulator(town):
        start_connect_time = time.time()
        path = os.path.join(base, f"{town}.xodr")
        simulator = CarlaSimulator(map_path=path, carla_map=town, timeout=120)
        end_connect_time = time.time()
        connect_elapsed_time = end_connect_time - start_connect_time
        print(f"CARLA connection established in {connect_elapsed_time:.2f} seconds.")
        return simulator, town, path

    yield _getCarlaSimulator

    if carla_process:
        subprocess.run("killall -9 CarlaUE4-Linux-Shipping", shell=True)


def test_throttle(getCarlaSimulator):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = '{town}'
        param time_step = 1.0/10

        model scenic.simulators.carla.model

        behavior DriveWithThrottle():
            while True:
                take SetThrottleAction(1)

        ego = new Car at (369, -326), with behavior DriveWithThrottle
        record ego.speed as CarSpeed
        terminate after 5 steps
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
                take SetThrottleAction(0), SetBrakeAction(1)

        behavior DriveThenBrake():
            do DriveWithThrottle() for 2 steps
            do Brake() for 6 steps

        ego = new Car at (369, -326),
            with blueprint 'vehicle.toyota.prius',
            with behavior DriveThenBrake
        record final ego.speed as CarSpeed
        terminate after 8 steps
    """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    finalSpeed = simulation.result.records["CarSpeed"]
    assert finalSpeed == pytest.approx(0.0, abs=1e-1)
