import scenic
import math
from scenic.simulators.carla import CarlaSimulator

code = f"""
param map = "assets/maps/CARLA/Town10HD_Opt.xodr"
model scenic.simulators.carla.model
param carla_map = "Town10HD_Opt"
param time_step = 1.0/10

behavior DriveFullThrottle():
    while True:
        take SetThrottleAction(1)

ego = new Car at (-42, -137),
    with behavior DriveFullThrottle,
    with blueprint "vehicle.nissan.patrol"


record initial ego.position as StartPos
record final   ego.position as EndPos
record final ego.speed as CarSpeed
# record ego.speed as SpeedLog 
terminate after 8 seconds
"""

scenario = scenic.scenarioFromString(code, mode2D=True)
scene, _ = scenario.generate()
#simulator = scenario.getSimulator()
simulator = CarlaSimulator(
    carla_map="Town10HD_Opt", 
    map_path="assets/maps/CARLA/Town10HD_Opt.xodr",
    timeout=60,
)
# simulation = simulator.simulate(scene, maxSteps=10)
simulation = simulator.simulate(scene,)

records   = simulation.result.records
print("FINAL SPEED: ", records["CarSpeed"])
# print("SPEED LOG: ", records["SpeedLog"])

start_pos = records['StartPos']  
end_pos   = records['EndPos']     

# get distance
dx   = end_pos[0] - start_pos[0]
dy   = end_pos[1] - start_pos[1]
dist = math.hypot(dx, dy)

print(f"Traveled {dist:.2f} m in 8 s")