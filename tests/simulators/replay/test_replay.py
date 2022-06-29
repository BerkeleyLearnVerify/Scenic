"""Tests for the ReplaySimulation Object, pythonically"""

import os
import re
import scenic
from scenic.simulators.newtonian import NewtonianSimulator, NewtonianReplaySimulation
import pytest
import pickle

# Mark all tests in this file as slow, since they require spawning a subprocess
pytestmark = pytest.mark.slow

def test_saving_simulation(loadLocalScenario):
    scenario = loadLocalScenario('basic.scenic')
    scene, _ = scenario.generate(maxIterations=1000)
    simulator = NewtonianSimulator()
    original_simulation = simulator.simulate(scene, maxSteps=3)
    original_simulation_result = original_simulation.result
    # try pickling and saving the result, first
    with open("original_simulation_result", 'wb') as fle:
        pickle.dump(original_simulation_result, fle)
    new_scenario = loadLocalScenario('basic_modified.scenic')
    new_scene, _ = new_scenario.generate(maxIterations=1000)
    replay_simulation = NewtonianReplaySimulation(new_scene, original_simulation_result, verbosity=5)
    replay_simulation.run(100)
    return replay_simulation