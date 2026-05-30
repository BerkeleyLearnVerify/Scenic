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


_DAY_MOD = 24*60*60

class Car(Vehicle):
    """A car.

    The default ``blueprint`` (see `CarlaActor`) is a uniform distribution over the
    blueprints listed in :obj:`scenic.simulators.carla.blueprints.carModels`.
    """
    blueprint: Uniform(*blueprints.carModels)

    origin: -1
    destination: -1

    def generateTrajectory(self,trajectory):
        self.trajectory = trajectory
        self.trajectory_is_active = False

    @property
    def isCar(self):
        return True

    def distanceToClosest(self, type: type) -> Object:
        """Compute the distance to the closest object of the given type.

        For example, one could write :scenic:`self.distanceToClosest(Car)` in a behavior.
        """
        objects = simulation().objects
        minDist = float('inf')
        for obj in objects:
            if not isinstance(obj, type):
                continue
            if obj.carla_actor_flag: # Fitler out vehicles
                d = distance from self to obj
                if 0 < d < minDist:
                    minDist = d
        return minDist
    

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
    behavior: CustomBubbleBehavior()


"""
Behaviors:
    A mix of default behaviors for the Interface
"""

behavior WaitBehavior():
    """
    Passive wait Behavior
    """
    while True:
        wait

behavior DisableAutoPilotThenDrive():
    """
    Disable Auto-pilot then initiate a secondary behavior
    """
    take SetAutoPilotAction(False)
    while True:
        do DriveAvoidingCollisions()

behavior SetAndFollowTrajectoryBehavior(target_speed, turn_speed):
    trajectory = self.trajectory
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=target_speed, turn_speed=turn_speed)
        
behavior StateBehavior(state_map, eval_func, verbose=False):
    """
    docstring for StateBehavior
    
    Alternates between state based behaviors

    :param state_map: Dictionary which maps each state to an intended action
    :type  state_map: dict[type(state) : Behavior()]
    :param eval_func: Fuction which maps an object to a given state
    :type eval_func:  Function
    """
    prev_state = eval_func()
    try:
        behavior = state_map[prev_state]
        do behavior
    interrupt when eval_func() != prev_state:
        prev_state = eval_func()
        behavior = state_map[prev_state]
        do behavior
        
behavior CustomBubbleBehavior():
    """
    Object will follow a random route starting at
    the origin zone and ending in the destination zone.
    If the object enters the bubble then it will envoke 
    followLaneBehavior or any general behavior
    """ 
    eval_func = lambda : self.carla_actor_flag
    state_map = {False: FollowRandomRoute(), True: DisableAutoPilotThenDrive()}
    do StateBehavior(state_map, eval_func)


behavior FollowSingleTrajectoryBehavior(target_speed = 10, trajectory = None, turn_speed=None):
    """
    Follows the given trajectory. The behavior terminates once the end of the trajectory is reached.
    If no trajectory is supplied, object will follow METSR proposed trajectory by default

    :param target_speed: Its unit is in m/s. By default, it is set to 10 m/s
    :param trajectory: It is a list of sequential lanes to track, from the lane that the vehicle is initially on to the lane it should end up on.
    """
    if trajectory is None:
        if not hasattr(self, "trajectory"):
             self.trajectory = None
        state_map = {False: WaitBehavior(), True: SetAndFollowTrajectoryBehavior(target_speed=target_speed, turn_speed=turn_speed)}
        state_func = lambda: bool(self.trajectory is not None and self.carla_actor_flag)
        while True:
            do StateBehavior(state_map, state_func)
    else:
        state_map = {False: SetAutoPilotAndWait(trajectory), True: FollowTrajectoryBehavior(target_speed=target_speed, turn_speed=turn_speed)}
        state_func = lambda: self.carla_actor_flag
        while True:
            do StateBehavior(state_map, state_fuc)
  
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
        take SetAutoPilotAction(True)

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




