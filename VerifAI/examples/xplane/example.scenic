# Load X-Plane world model
from scenic.simulators.xplane.model import *

# Length of simulation
param simulation_length = 30    # seconds to run the simulation for
param setup_time = 5        # seconds to wait before starting the simulation (for clouds, etc. to stabilize)

# Time of day
local_time = Range(6, 18)   # uniformly from 6 am to 6 pm
param 'sim/time/zulu_time_sec' = (local_time + 8) * 60 * 60    # Zulu time (GMT) in seconds

# No rain; uniformly random cloud type
param 'sim/weather/cloud_type[0]' = Uniform(0, 1, 2, 3, 4, 5)
param 'sim/weather/rain_percent' = 0

# Plane up to 8 m to left or right, facing down the runway
ego = Plane at Range(-8, 8) @ 0
