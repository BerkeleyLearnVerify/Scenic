from controller import Supervisor
import os
import scenic
from scenic.simulators.webots import WebotsSimulator

WEBOTS_RESULT_FILE_PATH=f"{os.path.dirname(__file__)}/../../../results.txt"

def send_results(data):
    with open(WEBOTS_RESULT_FILE_PATH, "w") as file:
        file.write(data)

supervisor = Supervisor()
simulator = WebotsSimulator(supervisor)

path = supervisor.getCustomData()
print(f"Loading Scenic scenario {path}")
scenario = scenic.scenarioFromFile(path)

scene, _ = scenario.generate()
simulation = simulator.simulate(scene, verbosity=2)
block_movements =simulation.result.records["BlockPosition"]
first_block_movement=block_movements[0]
last_block_movement=block_movements[-1]
blocks=[first_block_movement, last_block_movement]
supervisor.simulationQuit(status='finished')
send_results(str(blocks))