import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network
import random

Longitudinal_K_P = 0.8
Longitudinal_K_D = 0.2
Longitudinal_K_I = 0.7
Lateral_K_P = 0.2
Lateral_K_D = 0.1
Lateral_K_I = 0

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

random_seeds = [1, 10, 20, 25, 32, 37, 41, 46, 50, 53, 66, 69]

for i in range(len(random_seeds)):
    random.seed(random_seeds[i])
    scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
    scene, _ = scenario.generate(maxIterations=1000)
    # segments = scenario.params['segments']
    simulation = simulator.simulate(scene)
