from scenic.simulators.metadrive.simulator import MetaDriveSimulator 
from scenic.domains.driving.model import *
from scenic.simulators.metadrive.actions import *
import pathlib

# TODO: research on open drive conversion for metadrive
map_town = pathlib.Path(globalParameters.map).stem
sumo_map = pathlib.Path(globalParameters.sumo_map).stem

param map = map_town
param sumo_map = sumo_map
param timestep = 0.1
param render = 1

# (xmin, ymin), (xmax, ymax), _ = road.AABB
# param map_center_offset = ...

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    sumo_map=globalParameters.sumo_map,
)

class MetaDriveActor(DrivingObject):
    """Abstract class for MetaDrive objects.

    Properties:
        metaDriveActor (dynamic): Set during simulations to the ``metaDrive.Actor`` representing this
            object.
        blueprint (str): Identifier of the MetaDrive blueprint specifying the type of object.
        rolename (str): Can be used to differentiate specific actors during runtime. Default
            value ``None``.
    """
    metaDriveActor: None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def setPosition(self, pos):
        self.metaDriveActor.last_position = pos
        pass

    def setVelocity(self, vel):
        self.metaDriveActor.before_step([0, vel])


class Vehicle(Vehicle, Steers, MetaDriveActor):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.metaDriveActor.before_step([0, throttle])

    def setSteering(self, steering):
        self.metaDriveActor.before_step([steering, 0])

    def setBraking(self, braking):
        self.metaDriveActor.before_step([0, -braking])

    # def setHandbrake(self, handbrake):
    #     self.control.hand_brake = handbrake

    # def setReverse(self, reverse):
    #     self.control.reverse = reverse

class Car(Vehicle):
    @property
    def isCar(self):
        return True

# require ego
# try:
#     ego
# except NameError:
#     raise NameError("The 'ego' variable is not defined in the .scenic file. Please define an ego vehicle.")
