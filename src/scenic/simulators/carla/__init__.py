"""Scenic world model for the CARLA driving simulator.

This model is designed to be used with the CARLA interface to the VerifAI toolkit.
See the `VerifAI repository <https://github.com/BerkeleyLearnVerify/VerifAI>`_ for
further documentation and examples.

The model currently supports vehicles, pedestrians, and props.
Vehicles have an ``agent`` parameter, which specifies the agent to be used to control
the vehicle.

In addition, the model uses several global parameters to control weather (descriptions
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

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree: _autosummary

   model
   map
   interface
   car_models
   prop_models
"""
