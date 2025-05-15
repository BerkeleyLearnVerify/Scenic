from verifai.scenic_server import ScenicServer
from verifai.samplers import ScenicSampler
from verifai.monitor import specification_monitor
from verifai.falsifier import generic_falsifier
from dotmap import DotMap
scenic_sampler = ScenicSampler.fromScenicCode("""\
model scenic.simulators.newtonian.model
ego = new Object with velocity (0, Range(5, 15))
other = new Object at (5, 0), with velocity (-10, 10)
terminate after 2 seconds
record final (distance to other) as dist
""")

class scenic_spec(specification_monitor):
        def __init__(self):
                def specification(simulation):
                        return simulation.result.records['dist'] > 1
                super().__init__(specification)

falsifier = generic_falsifier(
        sampler=scenic_sampler,
        monitor=scenic_spec(),
        falsifier_params=DotMap(n_iters=2),
        server_class=ScenicServer
)
falsifier.run_falsifier()
print("FINISHED FALSIFICATION")
