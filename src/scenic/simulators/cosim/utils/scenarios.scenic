from scenic.simulators.metsr.traffic_flows import *


"""Library of useful scenarios for composing CoSim Scenic programs"""
scenario GeneratePrivateTrip(origin, destination, name=None):
    if name != None:
        new NPCCar with origin origin, with destination destination, with name name
    else:
        new NPCCar with origin origin, with destination destination
    terminate after 1 steps


"""Generic traffic Stream which spawns new vehicles according to the probabilities defined by traffic_flow"""
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
"""Constant Stream"""
scenario ConstantTrafficStream(origin, destination, num_vehicles, stime=None, etime=None):
    compose:
        tf = ConstantTrafficFlow(num_vehicles, stime, etime)
        do TrafficStream(origin, destination, tf)

"""Normally distributed Stream"""
scenario NormalTrafficStream(origin, destination, num_vehicles, peak_time, stddev):
    compose:
        tf = NormalTrafficFlow(num_vehicles, peak_time, stddev)
        do TrafficStream(origin, destination, tf)

"""Two-way stream modeling incoming and outgoing traffic """
scenario CommuterTrafficStream(origin, destination, num_vehicles, 
    peak_time_1, peak_time_2, stddev):
    compose:
        tf1 = NormalTrafficFlow(num_vehicles, peak_time_1, stddev)
        tf2 = NormalTrafficFlow(num_vehicles, peak_time_2, stddev)
        do TrafficStream(origin, destination, tf1), TrafficStream(destination, origin, tf2)
