import scenic
from scenic.simulators.carla.simulator import CarlaSimulator
​
# CHANGEME
carla_world = 'Town01'
sc_file_path = 'test.sc'
​
# Create the Simulator
simulator = LGSVLSimulator(carla_world, address='localhost', port=8181)
​
# Load Scenic scenario
scenario = scenic.scenarioFromFile(sc_file_path)
​
# Compute number of time steps to run simulations
timestep = 1.0 / 30
maxSteps = 20.0 / timestep
​
# Sample configurations from the scenario and run simulations
itr = 0
while itr < 10:
    itr = itr + 1
    scene, _ = scenario.generate()
    simulation = simulator.createSimulation(scene)
    simulation.run(maxSteps)
    print('DONE')
