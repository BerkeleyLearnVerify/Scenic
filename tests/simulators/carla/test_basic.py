from scenic.core.distributions import InvalidScenarioError
import pytest


from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def test_map_param_parse(getAssetPath):
    mapPath = getAssetPath("maps/CARLA/Town01.xodr")
    code = f"""
        param map = r'{mapPath}'
        model scenic.simulators.carla.model
        ego = new Car
    """
    scenario = compileScenic(code, mode2D=True)
    assert scenario.params["carla_map"] == "Town01"

# Note: This will used the current cached version of the map, during testing this does not work with a cached 3D map
@pytest.mark.parametrize("use2DMap", [True, False])
def test_basic(loadLocalScenario, use2DMap):
    scenario = loadLocalScenario("basic.scenic", mode2D=use2DMap)
    scenario.generate(maxIterations=1000)

@pytest.mark.parametrize("use2DMap", [True, False])
def test_car_created(loadLocalScenario, use2DMap):
    scenario = loadLocalScenario("basic.scenic", mode2D=use2DMap)
    scene = sampleScene(scenario, maxIterations=1000)
    car = scene.egoObject
    assert car
    
# A test to ensure that cars cannot clip through each other without causing errors
def test_car_clipping_3D(getAssetPath):
    #pytest.importorskip("carla")
    mapPath = getAssetPath("maps/CARLA/Town01.xodr")
    code = f"""
        param map = r'{mapPath}'
        param carla_map = 'Town01'
        model scenic.domains.driving.model
        ego = new Car at (100,0,1), with regionContainedIn everywhere 
        car2 = new Car at (100,0,1), with regionContainedIn everywhere
    """
    with pytest.raises(InvalidScenarioError):
        compileScenic(code, mode2D=False)

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
