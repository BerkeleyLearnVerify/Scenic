from verifai.features.features import *
from verifai.falsifier import mtl_falsifier
from dotmap import DotMap

init_conditions = Struct({
        'x_init': Box([-0.45, 0.45]),
        'cruising_speed': Box([10.0, 20.0]),
        'theta_init': Box([-np.pi/4, np.pi/4])
    })

sample_space = {'init_conditions': init_conditions}

SAMPLERTYPE = 'ce'
MAX_ITERS = 20
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

specification = ["G(xdeviation)"]

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.verbosity = 1
falsifier_params.compute_error_table = True
falsifier_params.fal_thres = 0.0

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)


sampler_params = DotMap()
sampler_params.init_num = 2
falsifier_params.sampler_params = sampler_params

falsifier = mtl_falsifier(sample_space=sample_space, sampler_type=SAMPLERTYPE,
                             specification=specification, falsifier_params=falsifier_params,
                          server_options=server_options)
falsifier.run_falsifier()
analysis_params = DotMap()
analysis_params.k_closest_params.k = 4
analysis_params.random_params.count = 4
falsifier.analyze_error_table(analysis_params=analysis_params)

print("Falsified Samples")
print(falsifier.error_table.table)

print("Safe Samples")
print(falsifier.safe_table.table)


# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))


