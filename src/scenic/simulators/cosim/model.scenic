import pathlib
from scenic.simulators.carla.model import Vehicle, is2DMode

import scenic.simulators.carla.blueprints as blueprints
from scenic.simulators.carla.behaviors import *
from scenic.simulators.utils.colors import Color


from scenic.simulators.cosim.simulator import CosimSimulator
map_town = pathlib.Path(globalParameters.map).stem
param xml_path = pathlib.Path(globalParameters.xml_map)
param carla_map = map_town
param metsr_host = "localhost"
param metsr_port = 4000
param address = "10.0.0.122"
param carla_port = 2000
param metsr_map = "Data.properties.CARLA"
param timestep = 0.1
param snapToGroundDefault = is2DMode()


simulator CosimSimulator(
    metsr_host = globalParameters.metsr_host,
    metsr_port = globalParameters.metsr_port,
    address = globalParameters.address, 
    carla_port = globalParameters.carla_port,
    metsr_map = globalParameters.metsr_map,
    carla_map = map_town,
    xml_map = globalParameters.xml_path,
    map_path = globalParameters.map,
    timestep = globalParameters.timestep,
    bubble_size = 50
    )


class Car(Vehicle):
    """A car.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.carModels`.
    """
    blueprint: Uniform(*blueprints.carModels)
    destination: -1
    origin: -1

    @property
    def isCar(self):
        return True


class EgoCar(Car):
    """
    Special class for Ego 
    """
    carla_actor_flag: True



class NPCCar(Car):
    """
    An NPC car
    """
    carla_actor_flag: False
    trajectory: None
    behavior: DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12)
    # Default Carla Behavior
    # behavior: FollowLaneBehavior(self.trajectory)


    


