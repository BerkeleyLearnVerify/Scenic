import scenic
from scenic.simulators.newtonian import NewtonianSimulator
from scenic.domains.driving.roads import Network
import random
from scipy.optimize import differential_evolution
import numpy as np 

# Simulate function
def simulate_pid(params):
    Lateral_K_P, Lateral_K_D, Lateral_K_I = params

    network = Network.fromFile('./assets/maps/CARLA/Town05.xodr')
    simulator = NewtonianSimulator(
        render=False, network=network,
        Longitudinal_K_P=0.5,
        Longitudinal_K_D=0.1,
        Longitudinal_K_I=0.7,
        Lateral_K_P=Lateral_K_P,
        Lateral_K_D=Lateral_K_D,
        Lateral_K_I=Lateral_K_I
    )

    random_seeds = [5,6,12,15,16,18,19,22,24,26,39,45,59]

    # Variables to accumulate total distance and count of points
    total_distance = 0
    total_count = 0

    # Loop over each random seed
    for random_choice in random_seeds:
        random.seed(random_choice)
        count = 0  # Reset count for each run

        while count <= 0:
            try:
                scenario = scenic.scenarioFromFile('test.scenic', mode2D=True)
                scene, _ = scenario.generate(maxIterations=1000)
                simulation = simulator.simulate(scene)
                if simulation is None:
                    continue
                distances = simulation.result.records
                for points in distances.values():
                    for point in points:
                        _, y = point
                        total_distance += abs(y)
                        count += 1
                total_count += count  # Add count of this run to the total count
            except Exception as e:
                print(f"Invalid scenario regenerating: {e}")

    # Calculate the average distance from the centerline
    if total_count > 0:
        average_distance_from_centerline = total_distance / total_count
        print("Average distance from centerline:", average_distance_from_centerline)
    else:
        print("No valid points found to calculate average distance.")

    return average_distance_from_centerline

# Define the objective function to minimize
def objective_function(params):
    print("Running new simulation:")
    print("="*20)
    return simulate_pid(params)

# Define the parameter bounds for the optimization
# test only lateral, bound from 0, average all seeds
# try these optimizers as well: https://docs.scipy.org/doc/scipy/reference/optimize.html#global-optimization
# differential evolution
# long term, LQR controller instead of PID controller
param_bounds = [
    # (0.05, 2.0),  # Longitudinal_K_P bounds
    # (0.05, 2.0),  # Longitudinal_K_D bounds
    # (0.01, 2.0),  # Longitudinal_K_I bounds
    (0.0, 2.0),  # Lateral_K_P bounds
    (0.0, 2.0),  # Lateral_K_D bounds
    (0.0, 2.0)   # Lateral_K_I bounds
]

# Run optimization using Differential Evolution
print("Running Differential Evolution optimization")
result = differential_evolution(objective_function, param_bounds, strategy='best1bin', maxiter=200, popsize=15, tol=0.01, mutation=(0.5, 1), recombination=0.7, seed=0)

print(f"Optimal PID parameters: {result.x}")
print(f"Minimum deviation from centerline: {result.fun}")
