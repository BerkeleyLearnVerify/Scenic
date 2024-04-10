import re

import pytest

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling

# Suppress potential warning about missing the carla package
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.core.simulators.SimulatorInterfaceWarning"
)

def model_pedestrians(simulator, mapPath, town, pedestrianModels):
    for model in pedestrianModels:
        code = f"""
                param map = r'{mapPath}'
                param carla_map = '{town}'
                param time_step = 1.0/10

                model scenic.simulators.carla.model
                ego = new Car
                new Pedestrian with blueprint '{model}'
                terminate after 1 steps
            """
        scenario = compileScenic(code, mode2D=True)
        scene, _ = scenario.generate(maxIterations=10000)
        simulation = simulator.simulate(scene)
        obj = simulation.objects[1]
        assert obj.blueprint == model


def model_iteration(simulator, mapPath, town, modelNames, modelType):
    for model in modelNames:
        code = f"""
            param map = r'{mapPath}'
            param carla_map = '{town}'
            param time_step = 1.0/10

            model scenic.simulators.carla.model
            ego = new {modelType} with blueprint '{model}'
            terminate after 1 steps
        """
        scenario = compileScenic(code, mode2D=True)
        scene, _ = scenario.generate(maxIterations=10000)
        simulation = simulator.simulate(scene)
        obj = simulation.objects[0]
        assert obj.blueprint == model

def test_old_blue_prints(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import oldBlueprintNames 
    simulator, town, mapPath = getCarlaSimulator('Town01')

    for old_model,_ in oldBlueprintNames.items():
        code = f"""
            param map = r'{mapPath}'
            param carla_map = '{town}'
            param time_step = 1.0/10

            model scenic.simulators.carla.model
            ego = new Car with blueprint '{old_model}'
            terminate after 1 steps
        """
        scenario = compileScenic(code, mode2D=True)
        scene, _ = scenario.generate(maxIterations=10000)
        simulation = simulator.simulate(scene)
        obj = simulation.objects[0]
        assert obj.blueprint == old_model

def test_car_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import carModels 
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, carModels, 'Car')

def test_bicycle_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import bicycleModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, bicycleModels, 'Bicycle')

def test_motorcycles_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import motorcycleModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, motorcycleModels, 'Motorcycle')

def test_truck_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import truckModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, truckModels, 'Truck')

def test_trash_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import trashModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, trashModels, 'Trash')

def test_cone_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import coneModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, coneModels, 'Cone')

def test_debris_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import debrisModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, debrisModels, 'Debris')

def test_vending_machine_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import vendingMachineModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, vendingMachineModels, 'VendingMachine')

def test_chair_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import chairModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, chairModels, 'Chair')

def test_bus_stop_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import busStopModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, busStopModels, 'BusStop')

def test_advertisement_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import advertisementModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, advertisementModels, 'Advertisement')

def test_garbage_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import garbageModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, garbageModels, 'Garbage')

def test_container_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import containerModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, containerModels, 'Container')

def test_table_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import tableModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, tableModels, 'Table')

def test_barrier_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import barrierModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, barrierModels, 'Barrier')

def test_plant_pot_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import plantpotModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, plantpotModels, 'PlantPot')

def test_mailbox_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import mailboxModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, mailboxModels, 'Mailbox')

def test_gnome_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import gnomeModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, gnomeModels, 'Gnome')

def test_creased_box_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import creasedboxModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, creasedboxModels, 'CreasedBox')

def test_case_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import caseModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, caseModels, 'Case')

def test_box_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import boxModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, boxModels, 'Box')

def test_bench_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import benchModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, benchModels, 'Bench')

def test_barrel_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import barrelModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, barrelModels, 'Barrel')

def test_atm_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import atmModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, atmModels, 'ATM')

def test_kiosk_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import kioskModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, kioskModels, 'Kiosk')

def test_iron_plate_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import ironplateModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, ironplateModels, 'IronPlate')

def test_traffic_warning_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import trafficwarningModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_iteration(simulator, mapPath, town, trafficwarningModels, 'TrafficWarning')

def test_pedestrian_models(getCarlaSimulator):
    pytest.importorskip("carla")
    from scenic.simulators.carla.blueprints import walkerModels
    simulator, town, mapPath = getCarlaSimulator('Town01')
    model_pedestrians(simulator, mapPath, town, walkerModels)

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
