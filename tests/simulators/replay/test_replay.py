"""Tests for the ReplaySimulation Object, pythonically"""

import inspect
import os
import re
import scenic
from scenic.simulators.newtonian import NewtonianSimulator, NewtonianReplaySimulation
import pytest
import pickle

# Mark all tests in this file as slow, since they require spawning a subprocess
pytestmark = pytest.mark.slow

## Utilities

paramPattern = re.compile(r'\s*Parameter "p": (.*)$')

def test_replay_simulation():
    scenario = scenic.scenarioFromFile(os.getcwd() + '/basic.scenic')
    scene, _ = scenario.generate(maxIterations=1000)
    simulator = NewtonianSimulator()
    original_simulation = simulator.simulate(scene, maxSteps=3)
    original_simulation_result = original_simulation.result
    # try pickling and saving the result, first
    # with open("original_simulation_result", 'wb') as fle:
    #     pickle.dump(original_simulation_result, fle)
    # now, try loading the simulaton result and new scene into a Replay object
    new_scenario = scenic.scenarioFromFile(os.getcwd() + '/basic_modified.scenic')
    new_scene, _ = new_scenario.generate(maxIterations=1000)
    replay_simulation = NewtonianReplaySimulation(scene, original_simulation_result, verbosity=3)
    replay_simulation.run(100)
    return replay_simulation

test_replay_simulation()