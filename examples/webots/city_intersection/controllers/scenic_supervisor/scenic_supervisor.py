import random

from controller import Supervisor
import numpy

import scenic
from scenic.core.distributions import RejectionException
from scenic.simulators.webots import WebotsSimulator

supervisor = Supervisor()
simulator = WebotsSimulator(supervisor)

# CONSTANTS
SEED = 39

random.seed(SEED)
numpy.random.seed(SEED)

path = supervisor.getCustomData()
print(f"Loading Scenic scenario {path}")
scenario = scenic.scenarioFromFile(path)

while True:
    scene, _ = scenario.generate(maxIterations=float("inf"))
    print("Starting new simulation...")
    sim_results = simulator.simulate(scene, timestep=0.5, verbosity=2)

    if sim_results is None:
        print("Simulation was rejected. Trying again...")
    else:
        print("Terminating...")
        break

supervisor.simulationQuit(0)
