import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network
import random
from skopt import gp_minimize
import numpy as np

# Simulate function
def simulate_pid(params):
    Longitudinal_K_P, Longitudinal_K_D, Longitudinal_K_I, Lateral_K_P, Lateral_K_D, Lateral_K_I = params

    network = Network.fromFile('./assets/maps/CARLA/Town01.xodr')
    simulator = NewtonianSimulator(
        render=False, network=network,
        Longitudinal_K_P=Longitudinal_K_P,
        Longitudinal_K_D=Longitudinal_K_D,
        Longitudinal_K_I=Longitudinal_K_I,
        Lateral_K_P=Lateral_K_P,
        Lateral_K_D=Lateral_K_D,
        Lateral_K_I=Lateral_K_I
    )

    random_seeds = [10, 30]

    total_distance = 0
    count = 0

    for seed in random_seeds:
        random.seed(seed)
        scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
        scene, _ = scenario.generate(maxIterations=1000)
        simulation = simulator.simulate(scene)
        if simulation == None:
            continue
        distances = simulation.result.records

        for points in distances.values():
            for point in points:
                _, y = point
                total_distance += abs(y)
                count += 1

    print("count: ", count)
    print("total_distance: ", total_distance)
    # print("average_distance_from_centerline: ", total_distance / count)
    if count > 0:
        average_distance_from_centerline = total_distance / count
    else:
        average_distance_from_centerline = 20 #hardcode the max distance

    return average_distance_from_centerline

# Define the objective function to minimize
def objective_function(params):
    return simulate_pid(params)

# Initial PID controller constants
initial_params = [
    0.5,  # Longitudinal_K_P
    0.1,  # Longitudinal_K_D
    0.2,  # Longitudinal_K_I
    0.3,  # Lateral_K_P
    0.2,  # Lateral_K_D
    0.0   # Lateral_K_I
]

# Define the parameter bounds for the optimization
param_bounds = [
    (0.1, 2.0),  # Longitudinal_K_P bounds
    (0.1, 2.0),  # Longitudinal_K_D bounds
    (0.1, 2.0),  # Longitudinal_K_I bounds
    (0.1, 2.0),  # Lateral_K_P bounds
    (0.1, 2.0),  # Lateral_K_D bounds
    (0, 2.0)   # Lateral_K_I bounds
]

# Run optimization using Gaussian process minimization
print("Running Gaussian process minimization")
result = gp_minimize(objective_function, param_bounds, x0=initial_params, n_calls=150, random_state=0)

print(f"Optimal PID parameters: {result.x}")
print(f"Minimum deviation from centerline: {result.fun}")
