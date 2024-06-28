import pytest

try:
    import carla
except ModuleNotFoundError:
    pytest.skip("carla package not installed", allow_module_level=True)

from test_actions import getCarlaSimulator

from scenic.simulators.carla.blueprints import (
    advertisementModels,
    atmModels,
    barrelModels,
    barrierModels,
    benchModels,
    bicycleModels,
    boxModels,
    busStopModels,
    carModels,
    caseModels,
    chairModels,
    coneModels,
    containerModels,
    creasedboxModels,
    debrisModels,
    garbageModels,
    gnomeModels,
    ironplateModels,
    kioskModels,
    mailboxModels,
    motorcycleModels,
    plantpotModels,
    tableModels,
    trafficwarningModels,
    trashModels,
    truckModels,
    vendingMachineModels,
    walkerModels,
)
from tests.utils import compileScenic, sampleScene

pytest.mark.skip_blueprints = pytest.mark.skip(
    reason="Skipping test due to Carla memory leak issues"
)


@pytest.fixture(autouse=True)
def skip_tests_with_blueprints(request):
    if "blueprints" in request.node.name.lower():
        pytest.skip("Skipping test due to Carla memory leak issues")


def model_blueprint(simulator, mapPath, town, modelType, modelName):
    code = f"""
            param map = r'{mapPath}'
            param carla_map = '{town}'
            param time_step = 1.0/10

            model scenic.simulators.carla.model
            ego = new {modelType} with blueprint '{modelName}'
            terminate after 1 steps
        """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    obj = simulation.objects[0]
    assert obj.blueprint == modelName


@pytest.mark.parametrize("modelName", carModels)
def test_car_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Car", modelName)


@pytest.mark.parametrize("modelName", bicycleModels)
def test_bicycle_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Bicycle", modelName)


@pytest.mark.parametrize("modelName", motorcycleModels)
def test_motorcycle_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Motorcycle", modelName)


@pytest.mark.parametrize("modelName", truckModels)
def test_truck_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Truck", modelName)


@pytest.mark.parametrize("modelName", trashModels)
def test_trash_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Trash", modelName)


@pytest.mark.parametrize("modelName", coneModels)
def test_cone_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Cone", modelName)


@pytest.mark.parametrize("modelName", debrisModels)
def test_debris_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Debris", modelName)


@pytest.mark.parametrize("modelName", vendingMachineModels)
def test_vending_machine_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "VendingMachine", modelName)


@pytest.mark.parametrize("modelName", chairModels)
def test_chair_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Chair", modelName)


@pytest.mark.parametrize("modelName", busStopModels)
def test_bus_stop_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "BusStop", modelName)


@pytest.mark.parametrize("modelName", advertisementModels)
def test_advertisement_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Advertisement", modelName)


@pytest.mark.parametrize("modelName", garbageModels)
def test_garbage_blueprints(getCarlaSimulator, modelName):
    pytest.importorskip("carla")
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Garbage", modelName)


@pytest.mark.parametrize("modelName", containerModels)
def test_container_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Container", modelName)


@pytest.mark.parametrize("modelName", tableModels)
def test_table_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Table", modelName)


@pytest.mark.parametrize("modelName", barrierModels)
def test_barrier_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Barrier", modelName)


@pytest.mark.parametrize("modelName", plantpotModels)
def test_plant_pots_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "PlantPot", modelName)


@pytest.mark.parametrize("modelName", mailboxModels)
def test_mailbox_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Mailbox", modelName)


@pytest.mark.parametrize("modelName", gnomeModels)
def test_gnome_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Gnome", modelName)


@pytest.mark.parametrize("modelName", creasedboxModels)
def test_creased_box_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "CreasedBox", modelName)


@pytest.mark.parametrize("modelName", caseModels)
def test_case_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Case", modelName)


@pytest.mark.parametrize("modelName", boxModels)
def test_box_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Box", modelName)


@pytest.mark.parametrize("modelName", benchModels)
def test_bench_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Bench", modelName)


@pytest.mark.parametrize("modelName", barrelModels)
def test_barrel_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Barrel", modelName)


@pytest.mark.parametrize("modelName", atmModels)
def test_atm_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "ATM", modelName)


@pytest.mark.parametrize("modelName", kioskModels)
def test_kiosk_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Kiosk", modelName)


@pytest.mark.parametrize("modelName", ironplateModels)
def test_iron_plate_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "IronPlate", modelName)


@pytest.mark.parametrize("modelName", trafficwarningModels)
def test_traffic_warning_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "TrafficWarning", modelName)


@pytest.mark.parametrize("modelName", walkerModels)
def test_pedestrian_blueprints(getCarlaSimulator, modelName):
    simulator, town, mapPath = getCarlaSimulator("Town01")
    model_blueprint(simulator, mapPath, town, "Pedestrian", modelName)
