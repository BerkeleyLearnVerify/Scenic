
import math
import os.path
import sys

from dotmap import DotMap

from verifai.samplers import ScenicSampler
from verifai.scenic_server import ScenicServer
from verifai.falsifier import generic_falsifier
from verifai.monitor import specification_monitor, mtl_specification

# Load the Scenic scenario and create a sampler from it
if len(sys.argv) > 1:
    path = sys.argv[1]
else:
    path = os.path.join(os.path.dirname(__file__), 'newtonian/carlaChallenge2.scenic')
sampler = ScenicSampler.fromScenario(path, mode2D=True)

# Define the specification (i.e. evaluation metric) as an MTL formula.
# Our example spec will say that the ego object stays at least 5 meters away
# from all other objects.
class MyMonitor(specification_monitor):
    def __init__(self):
        self.specification = mtl_specification(['G safe'])
        super().__init__(self.specification)

    def evaluate(self, simulation):
        # Get trajectories of objects from the result of the simulation
        traj = simulation.result.trajectory

        # Compute time-stamped sequence of values for 'safe' atomic proposition;
        # we'll define safe = "distance from ego to all other objects > 5"
        safe_values = []
        for positions in traj:
            ego = positions[0]
            dist = min((ego.distanceTo(other) for other in positions[1:]),
                       default=math.inf)
            safe_values.append(dist - 5)
        eval_dictionary = {'safe' : list(enumerate(safe_values)) }

        # Evaluate MTL formula given values for its atomic propositions
        return self.specification.evaluate(eval_dictionary)

# Set up the falsifier
falsifier_params = DotMap(
    n_iters=5,
    verbosity=1,
    save_error_table=True,
    save_safe_table=True,
    # uncomment to save these tables to files; we'll print them out below
    # error_table_path='error_table.csv',
    # safe_table_path='safe_table.csv'
)
server_options = DotMap(maxSteps=100, verbosity=0)
falsifier = generic_falsifier(sampler=sampler,
                              monitor=MyMonitor(),
                              falsifier_params=falsifier_params,
                              server_class=ScenicServer,
                              server_options=server_options)

# Perform falsification and print the results
falsifier.run_falsifier()
print('Error table:')
print(falsifier.error_table.table)
print('Safe table:')
print(falsifier.safe_table.table)
