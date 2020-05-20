To use these programs with CARLA, run `python3.8 driver.py`. You can change parameters for map path, scenic script, address, port, and HUD in `driver.py`. See example Scenic programs for CARLA at `Scenic/examples/carla`.

---

Currently supported objects are vehicles, pedestrians, and props.

Weather parameters are described below. (Descriptions are from the CARLA Python API reference.)
* `cloudiness` (float)
   Weather cloudiness. It only affects the RGB camera sensor. Values range from 0 to 100.
* `precipitation` (float)
   Precipitation amount for controlling rain intensity. It only affects the RGB camera sensor. Values range from 0 to 100.
* `precipitation_deposits` (float)
   Precipitation deposits for controlling the area of puddles on roads. It only affects the RGB camera sensor. Values range from 0 to 100.
* `wind_intensity` (float)
   Wind intensity, it affects the clouds moving speed, the raindrop direction, and vegetation. This doesn't affect the car physics. Values range from 0 to 100.
* `sun_azimuth_angle` (float)
   The azimuth angle of the sun in degrees. Values range from 0 to 360 (degrees).
* `sun_altitude_angle` (float)
   Altitude angle of the sun in degrees. Values range from -90 to 90 (where 0 degrees is the horizon). 
