import scenic
import pygame

from scenic.simulators.carla.simulator import CarlaSimulator

# ==============================================================================
# -- Parameters: CHANGEME ------------------------------------------------------

carla_map = 'Town01'
sc_file_path = 'scripts/test.sc'

address = '127.0.0.1'
port = 2000

render = True  # for HUD visualization

numSimulations = 10
maxSteps = 100

# ==============================================================================

simulator = CarlaSimulator(carla_map, address=address, port=port, render=render)
scenario = scenic.scenarioFromFile(sc_file_path)

for _ in range(numSimulations):
    scene, __ = scenario.generate()
    simulation = simulator.createSimulation(scene)
    simulation.run(maxSteps)
    print('DONE')

pygame.quit()
