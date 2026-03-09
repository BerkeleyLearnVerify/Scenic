import pathlib
from scenic.simulators.carla.model import Vehicle, is2DMode

import scenic.simulators.carla.blueprints as blueprints
from scenic.simulators.carla.behaviors import *
from scenic.simulators.utils.colors import Color

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
    bubble_size = 100
    )

"""
What kind of behaviors:
    Sensor Data
        Add some functions for accessing simulator data

    Intersection -- Traffic Link / intersection

    Helpers for measuring traffic metrics

"""


class Car(Vehicle):
    """A car.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.carModels`.
    """
    blueprint: Uniform(*blueprints.carModels)
    trajectory: None  # Set a trajectory for Carla autopilot
    origin: -1
    destination: -1
    
    @property
    def isCar(self):
        return True


class EgoCar(Car):
    """
    Special class for Ego 
    """
    carla_actor_flag: True
    behavior: DriveAvoidingCollisions(target_speed=15, avoidance_threshold=12)
    interrupt: False

behavior EgoAttack():
    condition = True # overwrite this with some condition to initiate attack
    while True:
        if condition:
            self.interrupt = True
            # do --- define attacker bahavior 
        else:
            wait


class NPCCar(Car):
    """
    An NPC car
    """
    finished_route_check: False
    destination: -1
    origin: -1
    carla_actor_flag: False


scenario GeneratePrivateTrip(origin,destination):
    new NPCCar with origin origin, with destination destination
    terminate after 1 steps

scenario TrafficStream(origin, destination, traffic_flow):
    compose:
        while True:
            raw_prob_spawn = traffic_flow.expected_vehs(
                currentTOD(), currentTOD()+simulation().timestep)
            if raw_prob_spawn < 0 or raw_prob_spawn > 1:
                warnings.warn(f"raw_prob_spawn (={raw_prob_spawn}) fell outside [0,1] and will be clamped.")
            prob_spawn = min(1, max(raw_prob_spawn, 0))
            if Range(0,1) < prob_spawn:
                do GeneratePrivateTrip(origin, destination)
            else:
                wait

scenario ConstantTrafficStream(origin, destination, num_vehicles, stime=None, etime=None):
    compose:
        tf = ConstantTrafficFlow(num_vehicles, stime, etime)
        do TrafficStream(origin, destination, tf)

scenario NormalTrafficStream(origin, destination, num_vehicles, peak_time, stddev):
    compose:
        tf = NormalTrafficFlow(num_vehicles, peak_time, stddev)
        do TrafficStream(origin, destination, tf)

scenario CommuterTrafficStream(origin, destination, num_vehicles, 
    peak_time_1, peak_time_2, stddev):
    compose:
        tf1 = NormalTrafficFlow(num_vehicles, peak_time_1, stddev)
        tf2 = NormalTrafficFlow(num_vehicles, peak_time_2, stddev)
        do TrafficStream(origin, destination, tf1), TrafficStream(destination, origin, tf2)



