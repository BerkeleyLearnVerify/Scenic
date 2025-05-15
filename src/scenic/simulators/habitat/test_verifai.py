from verifai.samplers import ScenicSampler
from verifai.monitor import specification_monitor
from verifai.falsifier import mtl_falsifier
from verifai.falsifier import generic_falsifier
from verifai.scenic_server import ScenicServer
from dotmap import DotMap

# path = '/home/kxu/Scenic_habitat/src/scenic/simulators/habitat/demos/test_verifai.scenic'
path = './demos/test_verifai.scenic'
scenic_sampler = ScenicSampler.fromScenario(path)
# specification = ["G(collisioncone0 & collisioncone1 & collisioncone2)"]

# # The specification must assume specification_monitor class
# class confidence_spec(specification_monitor):
    # def __init__(self):
        # def specification(traj):
            # return traj['yTrue'] == traj['yPred']
        # super().__init__(specification)

# falsifier_params = DotMap(
        # n_iters=1000,   # Number of simulations to run (or None for no limit)
        # max_time=None,  # Time limit in seconds, if any
        # save_error_table=True,   # Record samples that violated the monitor/specification
        # save_good_samples=False,  # Don't record samples that satisfied the monitor/specification
        # fal_thres=0.5,    # Monitor return value below which a sample is considered a violation
        # sampler_params=None   # optional DotMap of sampler-specific parameters
# )

PORT = 8888
BUFSIZE = 4096
MAXREQS = 5

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)



# falsifier = generic_falsifier(
        # sampler=scenic_sampler,         # or scenic_sampler, etc. as above
        # monitor=confidence_spec(),
        # falsifier_params=falsifier_params,
        # server_options=server_options
# )
class scenic_spec(specification_monitor):
        def __init__(self):
                def specification(simulation):
                    # print(simulation.result.records['dist'])
                    return all([i[1] > 1 for i in simulation.result.records['dist']])
                super().__init__(specification)

falsifier = generic_falsifier(
        sampler=scenic_sampler,
        monitor=scenic_spec(),
        falsifier_params=DotMap(n_iters=5),
        server_class=ScenicServer
)
# print("RUNNING FALSIFIER")
falsifier.run_falsifier()
print("FALSIFICACTION COMPLETE")
