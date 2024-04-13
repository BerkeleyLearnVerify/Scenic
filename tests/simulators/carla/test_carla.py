from flaky import flaky
import pytest

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)


def preprocess_old_blueprint_names(original):
    d = {}
    for key, value in original.items():
        for sub_value in value:
            d[sub_value] = key
    return d


def model_blueprint(simulator, mapPath, town, modelType, modelName, newModelNames=None):
    code = f"""
            param map = r'{mapPath}'
            param carla_map = '{town}'
            param time_step = 1.0/10

            model scenic.simulators.carla.model
            ego = new {modelType} with blueprint '{modelName}'
            terminate after 1 steps
        """
    scenario = compileScenic(code, mode2D=True)
    scene, _ = scenario.generate(maxIterations=10000)
    simulation = simulator.simulate(scene)
    obj = simulation.objects[0]
    if newModelNames:
        assert obj.blueprint == newModelNames[modelName]
    else:
        assert obj.blueprint == modelName


from scenic.simulators.carla.blueprints import carModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", carModels)
def test_car_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Car", modelName)


from scenic.simulators.carla.blueprints import oldBlueprintNames

oldToNew = preprocess_old_blueprint_names(oldBlueprintNames)
oldModels = oldToNew.keys()


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", oldModels)
def test_old_blue_prints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Car", modelName, oldToNew)


from scenic.simulators.carla.blueprints import bicycleModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", bicycleModels)
def test_bicycle_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Bicycle", modelName)


from scenic.simulators.carla.blueprints import motorcycleModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", motorcycleModels)
def test_motorcycle_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Motorcycle", modelName)


from scenic.simulators.carla.blueprints import truckModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", truckModels)
def test_truck_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Truck", modelName)


from scenic.simulators.carla.blueprints import trashModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", trashModels)
def test_trash_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Trash", modelName)


from scenic.simulators.carla.blueprints import coneModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", coneModels)
def test_cone_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Cone", modelName)


from scenic.simulators.carla.blueprints import debrisModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", debrisModels)
def test_debris_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Debris", modelName)


from scenic.simulators.carla.blueprints import vendingMachineModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", vendingMachineModels)
def test_vending_machine_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "VendingMachine", modelName)


from scenic.simulators.carla.blueprints import chairModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", chairModels)
def test_chair_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Chair", modelName)


from scenic.simulators.carla.blueprints import busStopModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", busStopModels)
def test_bus_stop_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "BusStop", modelName)


from scenic.simulators.carla.blueprints import advertisementModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", advertisementModels)
def test_advertisement_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Advertisement", modelName)


from scenic.simulators.carla.blueprints import garbageModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", garbageModels)
def test_garbage_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Garbage", modelName)


from scenic.simulators.carla.blueprints import containerModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", containerModels)
def test_container_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Container", modelName)


from scenic.simulators.carla.blueprints import tableModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", tableModels)
def test_table_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Table", modelName)


from scenic.simulators.carla.blueprints import barrierModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", barrierModels)
def test_barrier_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Barrier", modelName)


from scenic.simulators.carla.blueprints import plantpotModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", plantpotModels)
def test_plant_pots_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "PlantPot", modelName)


from scenic.simulators.carla.blueprints import mailboxModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", mailboxModels)
def test_mailbox_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Mailbox", modelName)


from scenic.simulators.carla.blueprints import gnomeModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", gnomeModels)
def test_gnome_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Gnome", modelName)


from scenic.simulators.carla.blueprints import creasedboxModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", creasedboxModels)
def test_creased_box_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "CreasedBox", modelName)


from scenic.simulators.carla.blueprints import caseModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", caseModels)
def test_case_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Case", modelName)


from scenic.simulators.carla.blueprints import boxModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", boxModels)
def test_box_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Box", modelName)


from scenic.simulators.carla.blueprints import benchModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", benchModels)
def test_bench_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Bench", modelName)


from scenic.simulators.carla.blueprints import barrelModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", barrelModels)
def test_barrel_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Barrel", modelName)


from scenic.simulators.carla.blueprints import atmModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", atmModels)
def test_atm_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "ATM", modelName)


from scenic.simulators.carla.blueprints import kioskModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", kioskModels)
def test_kiosk_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Kiosk", modelName)


from scenic.simulators.carla.blueprints import ironplateModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", ironplateModels)
def test_iron_plate_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "IronPlate", modelName)


from scenic.simulators.carla.blueprints import trafficwarningModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", trafficwarningModels)
def test_traffic_warning_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "TrafficWarning", modelName)


from scenic.simulators.carla.blueprints import walkerModels


@pytest.mark.skip(reason="Skipping test due to Carla memory leak issues")
@pytest.mark.parametrize("modelName", walkerModels)
def test_pedestrian_blueprints(getCarlaSimulator, launchCarlaServer, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Pedestrian", modelName)


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
    scene, _ = scenario.generate(maxIterations=10)
    simulation = simulator.simulate(scene)
    records = simulation.result.records["CarSpeed"]
    assert records[len(records) // 2][1] < records[-1][1]


pytest.importorskip("carla")


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
