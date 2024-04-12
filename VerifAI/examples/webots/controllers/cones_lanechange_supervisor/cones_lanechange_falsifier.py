from verifai.falsifier import mtl_falsifier
from verifai.features.features import *
from dotmap import DotMap

control_params = Struct({
        'x_init': Box([-0.05, 0.05]),
        'cruising_speed': Box([10.0, 20.0]),
        'reaction_time': Box([0.7, 1.00])
    })
env_params = Struct({
        'broken_car_color': Box([0.5, 1], [0.25, 0.75], [0, 0.5]),
        'broken_car_rotation': Box([5.70, 6.28])
    })
cones_config = Struct({
        'traffic_cones_pos_noise': Box([-0.25, 0.25], [-0.25, 0.25], [-0.25, 0.25]),
        'traffic_cones_down_0': Categorical(*np.arange(5)),
        'traffic_cones_down_1': Categorical(*np.arange(5)),
        'traffic_cones_down_2': Categorical(*np.arange(5))
    })

sample_space = {'control_params':control_params, 'env_params':env_params,
                'cones_config':cones_config}

SAMPLERTYPE = 'ce'
MAX_ITERS = 20
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

specification = ["G(collisioncone0 & collisioncone1 & collisioncone2)"]

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.compute_error_table = True

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

falsifier = mtl_falsifier(sample_space=sample_space, sampler_type=SAMPLERTYPE,
                          specification=specification, falsifier_params=falsifier_params,
                          server_options=server_options)
falsifier.run_falsifier()

print("Unsafe samples: Error table")
print(falsifier.error_table.table)

# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))
