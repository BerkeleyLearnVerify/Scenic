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


def model_blueprint(simulator, mapPath, town, modelType, modelName):
    code = f"""
            param map = r'{mapPath}'
            param carla_map = '{town}'
            param time_step = 1.0/10

            model scenic.simulators.carla.model
            ego = new {modelType} with blueprint '{modelName}',
                at (-3.3, -68),
                with regionContainedIn None
            terminate after 1 steps
        """
    scenario = compileScenic(code, mode2D=True)
    scene = sampleScene(scenario)
    simulation = simulator.simulate(scene)
    obj = simulation.objects[0]
    assert obj.blueprint == modelName


model_data = {
    "Car": carModels,
    "Bicycle": bicycleModels,
    "Motorcycle": motorcycleModels,
    "Truck": truckModels,
    "Trash": trashModels,
    "Cone": coneModels,
    "Debris": debrisModels,
    "VendingMachine": vendingMachineModels,
    "Chair": chairModels,
    "BusStop": busStopModels,
    "Advertisement": advertisementModels,
    "Garbage": garbageModels,
    "Container": containerModels,
    "Table": tableModels,
    "Barrier": barrierModels,
    "PlantPot": plantpotModels,
    "Mailbox": mailboxModels,
    "Gnome": gnomeModels,
    "CreasedBox": creasedboxModels,
    "Case": caseModels,
    "Box": boxModels,
    "Bench": benchModels,
    "Barrel": barrelModels,
    "ATM": atmModels,
    "Kiosk": kioskModels,
    "IronPlate": ironplateModels,
    "TrafficWarning": trafficwarningModels,
    "Pedestrian": walkerModels,
}


@pytest.mark.parametrize(
    "modelType, modelName",
    [(type, name) for type, names in model_data.items() for name in names],
)
def test_model_blueprints(getCarlaSimulator, modelType, modelName):
    simulator, town, mapPath = getCarlaSimulator()
    model_blueprint(simulator, mapPath, town, modelType, modelName)
