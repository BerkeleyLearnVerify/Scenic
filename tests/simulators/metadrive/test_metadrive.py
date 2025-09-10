import os

import pytest

try:
    import metadrive

    from scenic.simulators.metadrive import MetaDriveSimulator
except ModuleNotFoundError:
    pytest.skip("MetaDrive package not installed", allow_module_level=True)

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic", mode2D=True)
    scenario.generate(maxIterations=1000)


@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic", mode2D=True))
    tryPickling(sampleScene(scenario, maxIterations=1000))


@pytest.fixture(scope="package")
def getMetadriveSimulator(getAssetPath):
    base = getAssetPath("maps/CARLA")

    def _getMetadriveSimulator(town):
        openDrivePath = os.path.join(base, f"{town}.xodr")
        sumoPath = os.path.join(base, f"{town}.net.xml")
        simulator = MetaDriveSimulator(sumo_map=sumoPath, render=False)
        return simulator, openDrivePath, sumoPath

    yield _getMetadriveSimulator


def test_throttle(getMetadriveSimulator):
    simulator, openDrivePath, sumoPath = getMetadriveSimulator("Town01")
    code = f"""
        param map = r'{openDrivePath}'
        param sumo_map = r'{sumoPath}'

        model scenic.simulators.metadrive.model

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
    speeds = simulation.result.records["CarSpeed"]
    assert speeds[len(speeds) // 2][1] < speeds[-1][1]


@pytest.mark.xfail(
    reason="Expected failure until MetaDrive uploads the next version on PyPI to fix the issue where cars aren't fully stopping."
)
def test_brake(getMetadriveSimulator):
    simulator, openDrivePath, sumoPath = getMetadriveSimulator("Town01")
    code = f"""
        param map = r'{openDrivePath}'
        param sumo_map = r'{sumoPath}'

        model scenic.simulators.metadrive.model

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
            with behavior DriveThenBrake
        record final ego.speed as CarSpeed
        terminate after 8 steps
    """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    finalSpeed = simulation.result.records["CarSpeed"]
    assert finalSpeed == pytest.approx(0.0, abs=1e-1)


def test_pedestrian_movement(getMetadriveSimulator):
    simulator, openDrivePath, sumoPath = getMetadriveSimulator("Town01")
    code = f"""
        param map = r'{openDrivePath}'
        param sumo_map = r'{sumoPath}'

        model scenic.simulators.metadrive.model

        behavior WalkForward():
            while True:
                take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(0.5)

        behavior StopWalking():
            while True:
                take SetWalkingSpeedAction(0)

        behavior WalkThenStop():
            do WalkForward() for 2 steps
            do StopWalking() for 2 steps

        ego = new Car at (30, 2)
        pedestrian = new Pedestrian at (50, 6), with behavior WalkThenStop

        record initial pedestrian.position as InitialPos
        record final pedestrian.position as FinalPos
        terminate after 4 steps
    """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    initialPos = simulation.result.records["InitialPos"]
    finalPos = simulation.result.records["FinalPos"]
    assert initialPos != finalPos
