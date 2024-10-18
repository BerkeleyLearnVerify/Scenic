from scenic.simulators.metsr.simulator import METSRSimulator

simulator METSRSimulator(
    host="localhost", 
    port=4000,
    map_name="Data.properties.CARLA"
    )

zone_2_center = (-0.0024190,-0.0000165,0)
zone_9_center = (0.0013876,-0.0000135, 0)

ego = new Object with origin 2, with destination 9

record ego.position as "pos"

terminate after 2000 steps
