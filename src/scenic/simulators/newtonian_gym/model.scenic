"""Scenic world model for the Newtonian simulator.

This is a completely generic model that does not assume the scenario takes
place in a road network (unlike `scenic.simulators.newtonian.driving_model`).
"""

from scenic.simulators.newtonian.simulator import NewtonianSimulator    # for use in scenarios

if 'render' not in globalParameters:
    render = True
else:
    render = globalParameters.render
simulator NewtonianSimulator(None, render=render)
