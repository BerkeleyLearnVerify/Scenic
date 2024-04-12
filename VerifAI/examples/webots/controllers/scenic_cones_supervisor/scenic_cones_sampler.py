from verifai.samplers.scenic_sampler import ScenicSampler
from dotmap import DotMap
from verifai.falsifier import generic_falsifier


path_to_scenic_file = 'lane_cones.scenic'
sampler = ScenicSampler.fromScenario(path_to_scenic_file)

MAX_ITERS = 20
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.save_error_table = False
falsifier_params.save_good_samples = False

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

falsifier = generic_falsifier(sampler=sampler, sampler_type='scenic',
                              falsifier_params=falsifier_params,
                              server_options=server_options)

falsifier.run_falsifier()

print("Scenic Samples")
for i in falsifier.samples.keys():
    print("Sample: ", i)
    print(falsifier.samples[i])


# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))
