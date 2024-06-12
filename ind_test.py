import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network
import random

#Optimal PID parameters: [1.068748220730243, 1.52882594227562, 1.3429667894860122, 0.48262808460121065, 0.1993880712528837, 0.9565778132913583]
Longitudinal_K_P = 1.068748220730243
Longitudinal_K_D = 1.52882594227562
Longitudinal_K_I = 1.3429667894860122
Lateral_K_P = 0.25
Lateral_K_D = 0.19
Lateral_K_I = 0.1

# lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
# lat_controller = PIDLateralController(K_P=0.1, K_D=0.1, K_I=0.02, dt=dt)

network = Network.fromFile('./assets/maps/CARLA/Town01.xodr')
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


random.seed(30)
scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
scene, _ = scenario.generate(maxIterations=1000)
# segments = scenario.params['segments']
simulation = simulator.simulate(scene)

distances = simulation.result.records
print(distances)