from scenic.simulators.cosim.simulator import CosimSimulator

param metsr_host = "localhost"
param metsr_port = 4000
param carla_host = "localhost"
param carla_port = 2000
param metsr_map = "Data.properties.CARLA"
param carla_map = "Town05"
param timestep = 0.1
param map_path = f"../../assets/maps/{globalParameters.carla_map}.xodr"

simulator CosimSimulator(
    metsr_host = globalParameters.metsr_host,
    metsr_port = globalParameters.metsr_port,
    carla_host = globalParameters.carla_host, 
    carla_port = globalParameters.carla_port,
    metsr_map = globalParameters.metsr_map,
    carla_map = globalParameters.carla_map,
    map_path = globalParameters.map_path,
    timestep = globalParameters.timestep,
    )


class PrivateCar:
    carla_actor_flag = False # all vehicles are instantiated assuming they are outside of carla
    pass



scenario GeneratePrivateTrip(origin, destination):
    new PrivateCar with origin origin, with destination destination
    terminate after 1 steps