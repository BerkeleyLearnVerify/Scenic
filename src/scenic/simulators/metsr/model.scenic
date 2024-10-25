from scenic.simulators.metsr.simulator import METSRSimulator
from scenic.simulators.metsr.traffic_flows import *

# Default start time is 6:00 AM
param startTime = 6*60*60
param timestep = 0.1

simulator METSRSimulator(
    host="localhost", 
    port=4000,
    map_name="Data.properties.CARLA",
    timestep=globalParameters.timestep
    )

# Internal time in seconds
_internalTime = globalParameters.startTime

_DAY_MOD = 24*60*60


class PrivateCar:
    pass

scenario GeneratePrivateTrip(origin, destination):
    new PrivateCar with origin origin, with destination destination
    terminate after 1 steps

scenario TrafficStream(origin, destination, traffic_flow):
    compose:
        while True:
            if Range(0,1) < traffic_flow.probSpawn(_internalTime%_DAY_MOD, globalParameters.timestep):
                do GeneratePrivateTrip(origin, destination)
            else:
                wait

scenario ConstantTrafficStream(origin, destination, vph):
    compose:
        do TrafficStream(origin, destination, ConstantTrafficFlow(vph))
