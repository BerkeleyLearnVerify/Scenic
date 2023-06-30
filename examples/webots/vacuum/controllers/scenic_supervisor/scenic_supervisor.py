from datetime import datetime
import json
from pathlib import Path
import random
import time

from controller import Supervisor
import numpy

import scenic
from scenic.simulators.webots import WebotsSimulator

# HELPER DATA STRUCTURES & FUNCTIONS


def getFilename(duration: int, numToys: int, iteration: int) -> str:
    return f"vacuum_d{str(duration).zfill(2)}_t{str(numToys).zfill(3)}_i{str(iteration).zfill(2)}.json"


# CONSTANTS

SEED = 44
# how many times perform simulations?
ITERATION = 25  # Value used in CAV23 paper is 25
# how long to run simulation for (minutes)
DURATION = 5  # Value used in CAV23 paper is 5
# number of toys to simulate
NUM_TOYS_LIST = [0, 1, 2, 4, 8, 16]

# save logs to `logs`
output_dir = Path(__file__).resolve().parent.parent.parent / "logs"
output_dir.mkdir(parents=True, exist_ok=True)

supervisor = Supervisor()
path = supervisor.getCustomData()

for numToys in NUM_TOYS_LIST:
    print(f"Loading Scenic scenario {path}")
    params = {
        "numToys": numToys,  # how many toys to place
        "duration": DURATION,  # how long to run simulation for (minute)
    }
    scenario = scenic.scenarioFromFile(path, params=params)
    for i in range(ITERATION):
        simulator = WebotsSimulator(supervisor)

        iter_seed = SEED + i
        filename = getFilename(duration=DURATION, numToys=numToys, iteration=i + 1)
        if (output_dir / filename).is_file():
            # print(f"Skipping simulation for {numToys} toys, #{i + 1} iteration because the file already exists")
            continue
        print("Calculate", filename)

        ts = datetime.now().strftime("%m-%d-%Y-%H-%M-%S")

        random.seed(iter_seed)
        numpy.random.seed(iter_seed)

        scene, _ = scenario.generate(maxIterations=float("inf"))
        sim_results = simulator.simulate(scene, timestep=0.25, verbosity=2).result

        s = json.dumps({"params": params, "results": sim_results.records}, indent=4)
        with open(output_dir / filename, "x") as f:
            f.write(s)

        time.sleep(1)

        supervisor.worldReload()

supervisor.simulationQuit(0)
