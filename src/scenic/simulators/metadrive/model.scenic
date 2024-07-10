from scenic.simulators.metadrive.simulator import MetaDriveSimulator 
from scenic.domains.driving.model import *
import pathlib

# TODO: research on open drive conversion for metadrive
map_town = pathlib.Path(globalParameters.map).stem
param map = map_town
param timestep = 0.1
param render = 0

simulator MetaDriveSimulator(
    timestep=float(globalParameters.timestep),
    render=bool(globalParameters.render),
    metadrive_map=globalParameters.map,
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
    rolename: None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def setPosition(self, pos):
        self.metaDriveActor.set_location(pos)

    def setVelocity(self, vel):
        self.metaDriveActor.set_velocity(vel, 1, in_local_frame=True)


class Vehicle(Vehicle, MetaDriveActor):
    """Abstract class for steerable vehicles."""

    def setThrottle(self, throttle):
        self.metaDriveActor._apply_throttle_brake(0)

    def setSteering(self, steering):
        self.control.steer = steering

    def setBraking(self, braking):
        self.control.brake = braking

    def setHandbrake(self, handbrake):
        self.control.hand_brake = handbrake

    def setReverse(self, reverse):
        self.control.reverse = reverse

class Car:
    blueprint: None
    @property
    def isCar(self):
        return True
