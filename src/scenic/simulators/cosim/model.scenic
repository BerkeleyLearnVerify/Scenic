import pathlib
from scenic.simulators.carla.model import Vehicle, is2DMode

import scenic.simulators.carla.blueprints as blueprints
from scenic.simulators.carla.behaviors import *
from scenic.simulators.utils.colors import Color

from .utils.CoSimActions import *
from .utils.scenarios import *

from scenic.simulators.metsr.traffic_flows import *

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
param bubble_size = 50


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
    bubble_size = globalParameters.bubble_size
    )

param startTime = 6*60*60
param verbose=False

"""
What kind of behaviors:
    Sensor Data
        Add some functions for accessing simulator data

    Intersection -- Traffic Link / intersection

    Helpers for measuring traffic metrics

"""
_DAY_MOD = 24*60*60

class Car(Vehicle):
    """A car.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.carModels`.
    """
    blueprint: Uniform(*blueprints.carModels)

    origin: -1
    destination: -1

    @property
    def isCar(self):
        return True

behavior FollowRandomRoute():
    """
    Object will follow a random route starting from its 
    spawn position to a random target road. Low level controls will be handled 
    via the METSR driving module and CARLA autopilot

    NOTE: Random route behaviors are restricted to non-ego vehicles 
          This is due to the fact that CARLA may remove deadlocked or
          non-moving vehicles from the simulation
    """
    while True:
        take SetAutoPilotAction()
    

class EgoCar(Car):
    """
    Car which defines the corresponding bubble location 
    based on its current position
    """
    carla_actor_flag = True
    behavior: DriveAvoidingCollisions()


class NPCCar(Car, BackgroundDriver):
    """
    A Non-Ego vehicle which has no effect on the defined bubble region

    :param carla_actor_flag: Dynamic flag representing vehicles presence in the
                             Cosimluated region. This flag will be set internally and should
                             not be modifed 
    :type: carla_actor_flag: bool

    :param behavior: Actor behavior
    :type behavior: Scenic Behavior
    """
    carla_actor_flag = False
    behavior: FollowRandomRoute()


behavior CustomBubbleBehavior():
    """
    Object will follow a random route starting at
    the origin zone and ending in the destination zone.
    If the object enters the bubble then it will envoke 
    followLaneBehavior or any general CARLA behavior
    """    
    while True:
        if self.carla_actor_flag:
            do FollowLaneBehavior()
        else:
            do FollowRandomRoute()



behavior EgoAttack():
    condition = True # overwrite this with some condition to initiate attack
    while True:
        if condition:
            self.interrupt = True
            # do --- define attacker bahavior 
        else:
            wait

def currentTOD():
    return (simulation().currentTime * simulation().timestep + globalParameters.startTime)%_DAY_MOD




