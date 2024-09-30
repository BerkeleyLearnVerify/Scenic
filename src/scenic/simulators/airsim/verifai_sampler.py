from verifai.samplers.scenic_sampler import ScenicSampler
from verifai.falsifier import generic_falsifier
from dotmap import DotMap
from verifai.scenic_server import ScenicServer
from verifai.monitor import specification_monitor
import math

path_to_scenic_script = 'examples/airsim/demoDrone.scenic'
sampler = ScenicSampler.fromScenario(path_to_scenic_script)

class confidence_spec(specification_monitor):
    def __init__(self):
        def specification(simulation):
            for traj in simulation.result.trajectory:
                # success condition is if the drone is within 7 meters of the target
                dist = simulation.result.records['dist']
                if dist < 7: 
                    return True
            print("ADVERSARY NOT FOUND")
            return False
            
        super().__init__(specification)

MAX_ITERS = 3
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096
falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS # Number of simulations to run (or None for no limit)
falsifier_params.max_time = None # Time limit in seconds, if any
falsifier_params.save_error_table = True # Record samples that violated the monitor/specification
falsifier_params.save_good_samples = True # Don't record samples that satisfied the monitor/specification
falsifier_params.error_table_path = 'error_table.csv'
falsifier_params.safe_table_path = 'safe_table.csv'
falsifier_params.fal_thres = 0.5 # Monitor return value below which a sample is considered a violation
falsifier_params.sampler_params = None   # optional DotMap of sampler-specific parameters

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS, verbosity=1)

falsifier = generic_falsifier(sampler=sampler,
                              falsifier_params=falsifier_params,
                              monitor=confidence_spec(),
                              server_class=ScenicServer,
                              server_options=server_options)
falsifier.run_falsifier()
print("Sampler Complete")
