"""Scenic world model for traffic scenarios in CARLA.

The model currently supports vehicles, pedestrians, and props.

The model uses several global parameters to control weather (descriptions
are from the CARLA Python API reference):

    * ``cloudiness`` (float):
      Weather cloudiness. It only affects the RGB camera sensor. Values range from 0 to 100.
    * ``precipitation`` (float):
      Precipitation amount for controlling rain intensity. It only affects the RGB camera sensor. Values range from 0 to 100.
    * ``precipitation_deposits`` (float):
      Precipitation deposits for controlling the area of puddles on roads. It only affects the RGB camera sensor. Values range from 0 to 100.
    * ``wind_intensity`` (float):
      Wind intensity, it affects the clouds moving speed, the raindrop direction, and vegetation. This doesn't affect the car physics. Values range from 0 to 100.
    * ``sun_azimuth_angle`` (float):
      The azimuth angle of the sun in degrees. Values range from 0 to 360 (degrees).
    * ``sun_altitude_angle`` (float):
      Altitude angle of the sun in degrees. Values range from -90 to 90 (where 0 degrees is the horizon).
"""

from scenic.domains.driving.model import *

from scenic.simulators.newtonian.behaviors import *
from scenic.simulators.utils.colors import Color

try:
    from scenic.simulators.newtonian.simulator import NewtonianSimulator    # for use in scenarios
    from scenic.simulators.newtonian.actions import *
except ModuleNotFoundError as e:
    # for convenience when testing without the carla package
    import warnings
    print(e)
    warnings.warn('the "carla" package is not installed; '
                  'will not be able to run dynamic simulations')

    def NewtonianSimulator(*args, **kwargs):
        raise RuntimeError('the "carla" package is required to run simulations '
                           'from this scenario')

if 'carla_map' not in globalParameters:
    raise RuntimeError('need to specify map before importing CARLA model '
                       '(set the global parameter "carla_map")')
if 'render' not in globalParameters:
    render = False
else:
    render = globalParameters.render
simulator NewtonianSimulator(globalParameters.carla_map, render=render)

class NewtonianActor(DrivingObject):

    position: None
    velocity: None
    throttle: 0
    steer: 0
    brake: 0
    hand_brake: 0
    reverse: 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._control = None    # used internally to accumulate control updates

    def setPosition(self, pos, elevation):
        self.position = pos

    def setVelocity(self, vel):
        self.velocity = vel

    def setThrottle(self, throttle):
        self.throttle = throttle

    def setSteering(self, steering):
        self.steer = steering

    def setBraking(self, braking):
        self.brake = braking

    def setHandbrake(self, handbrake):
        self.hand_brake = handbrake

    def setReverse(self, reverse):
        self.reverse = reverse

class Vehicle(Vehicle, NewtonianActor):

    pass

class Car(Vehicle):
    pass