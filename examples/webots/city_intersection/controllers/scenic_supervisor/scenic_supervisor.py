import random
import numpy

from controller import Supervisor

import scenic
from scenic.simulators.webots import WebotsSimulator
from scenic.core.distributions import RejectionException

supervisor = Supervisor()
simulator = WebotsSimulator(supervisor, timestep=0.5)

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
    sim_results = simulator.simulate(scene, verbosity=2)

    if sim_results is None:
        print("Simulation was rejected. Trying again...")
    else:
        print("Terminating...")
        break

supervisor.simulationQuit(0)
