import pytest

from scenic.core.simulators import SimulationCreationError
from scenic.simulators.metadrive import MetaDriveSimulator
from tests.utils import compileScenic

# Suppress potential warning about missing the metadrive package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic", mode2D=True)
    scenario.generate(maxIterations=1000)


def test_simulator_import():
    pytest.importorskip("metadrive")
    from scenic.simulators.metadrive import MetaDriveSimulator


def test_no_metadrive_agents_present(getAssetPath):
    mapPath = getAssetPath("maps/CARLA/Town01.xodr")
    sudoMapPath = getAssetPath("maps/CARLA/Town01.net.xml")
    code = f"""
        param map = r'{mapPath}'
        param sumo_map = r'{sudoMapPath}'
        model scenic.simulators.metadrive.model
    """
    with pytest.raises(SystemExit):
        with pytest.raises(SimulationCreationError):
            scenario = compileScenic(code, mode2D=True)
            scene, _ = scenario.generate(maxIterations=1)
            simulator = MetaDriveSimulator()
            _ = simulator.simulate(scene, maxSteps=2)
