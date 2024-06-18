import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network
import random

#lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
Longitudinal_K_P = 0.5
Longitudinal_K_D = 0.1
Longitudinal_K_I =  0.7

#Optimal PID parameters: [1.5552822622485505, 0.0, 1.5611843489824762]
Lateral_K_P = 0.42239561
Lateral_K_D = 0.12354368
Lateral_K_I = 0.72921939

# lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
# lat_controller = PIDLateralController(K_P=0.1, K_D=0.1, K_I=0.02, dt=dt)

network = Network.fromFile('./assets/maps/CARLA/Town05.xodr')
simulator = NewtonianSimulator(
    render=True, network=network,
    Longitudinal_K_P=Longitudinal_K_P,
    Longitudinal_K_D=Longitudinal_K_D,
    Longitudinal_K_I=Longitudinal_K_I,
    Lateral_K_P=Lateral_K_P,
    Lateral_K_D=Lateral_K_D,
    Lateral_K_I=Lateral_K_I
)

# random_seeds = [1, 10, 20, 25, 37, 41, 46, 50, 53, 66, 69]

# for i in range(len(random_seeds)):
#     random.seed(random_seeds[i])
#     scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
#     scene, _ = scenario.generate(maxIterations=1000)
#     # segments = scenario.params['segments']
#     simulation = simulator.simulate(scene)


random_seeds = [5,6,12,15,16,18,19,22,24,26,39,45,59]
# random.seed(10)
# random.seed(59)
for r in random_seeds:
    random.seed(r)
    # scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
    scenario = scenic.scenarioFromFile('examples/driving/badlyParkedCarPullingIn.scenic', mode2D=True)
    scene, _ = scenario.generate(maxIterations=1000)
    # segments = scenario.params['segments']
    simulation = simulator.simulate(scene)

# distances = simulation.result.records
# print(distances)