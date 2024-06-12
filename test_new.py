import random
import numpy as np
from bayes_opt import BayesianOptimization
import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network

# Define the simulation function to evaluate the PID controller performance
def simulate_pid(Longitudinal_K_P, Longitudinal_K_D, Longitudinal_K_I, Lateral_K_P, Lateral_K_D, Lateral_K_I):
    network = Network.fromFile('./assets/maps/CARLA/Town01.xodr')
    simulator = NewtonianSimulator(
        render=False,  # Set to False to speed up the optimization
        network=network,
        Longitudinal_K_P=Longitudinal_K_P,
        Longitudinal_K_D=Longitudinal_K_D,
        Longitudinal_K_I=Longitudinal_K_I,
        Lateral_K_P=Lateral_K_P,
        Lateral_K_D=Lateral_K_D,
        Lateral_K_I=Lateral_K_I
    )
    
    random_seeds = [1, 10, 20, 25, 37, 41, 46, 50, 53, 66, 69]
    total_distance = 0
    count = 0

    for seed in random_seeds:
        random.seed(seed)
        scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
        scene, _ = scenario.generate(maxIterations=1000)
        simulation = simulator.simulate(scene, maxSteps=60)
        if simulation == None:
            print("No simulation created")
            continue
        distances = simulation.result.records
        
        for points in distances.values():
            for point in points:
                _, y = point
                total_distance += abs(y)
                count += 1

    if count > 0:
        average_distance_from_center_lane = total_distance / count
    else:
        average_distance_from_center_lane = float('inf')

    # The objective is to minimize the average distance from the center lane
    return -average_distance_from_center_lane

# Define the bounds of the PID controller constants
pbounds = {
    'Longitudinal_K_P': (0.45, 0.6),
    'Longitudinal_K_D': (0.05, 0.15),
    'Longitudinal_K_I': (0.15, 0.25),
    'Lateral_K_P': (0.25, 0.4),
    'Lateral_K_D': (0.1, 0.3),
    'Lateral_K_I': (0.0, 0.2)
}

# Initialize the Bayesian optimizer
optimizer = BayesianOptimization(
    f=simulate_pid,
    pbounds=pbounds,
    random_state=1,
)

# Perform the optimization
optimizer.maximize(
    init_points=5,
    n_iter=30,
)

# Print the best result
print("Best PID parameters found:")
print(optimizer.max)