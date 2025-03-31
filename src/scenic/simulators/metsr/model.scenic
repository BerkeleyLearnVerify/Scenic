import warnings

from scenic.simulators.metsr.simulator import METSRSimulator
from scenic.simulators.metsr.traffic_flows import *

# Default start time is 6:00 AM
param startTime = 6*60*60
param timestep = 1
param simTimestep = 0.1

simulator METSRSimulator(
    host="localhost", 
    port=4000,
    map_name="Data.properties.CARLA",
    timestep=globalParameters.timestep,
    sim_timestep=globalParameters.simTimestep,
    verbose=True
    )

_DAY_MOD = 24*60*60

def currentTOD():
    return (simulation().currentTime * simulation().timestep + globalParameters.startTime)%_DAY_MOD

class PrivateCar:
    pass

scenario GeneratePrivateTrip(origin, destination):
    new PrivateCar with origin origin, with destination destination
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
