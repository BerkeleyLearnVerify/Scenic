from verifai.features.features import *
from verifai.falsifier import mtl_falsifier
from dotmap import DotMap

try:
    import gym
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires OpenAI-gym to be installed')


env_id = 'CartPole-v0'
cp = gym.make(env_id)

init_conditions = Struct({
        'x_init': Box([-0.5, 0.05]),
        'theta_init': Box([-0.1, 0.1]),
        'length':Box([cp.env.length - 0.25, cp.env.length+0.25]),
        'masscart': Box([cp.env.masscart-0.25, cp.env.masscart+0.25]),
        'masspole': Box([cp.env.masspole-0.025, cp.env.masspole+0.025])
    })

training_conditions = Struct({
    'num_timesteps': DiscreteBox([1000, 10000]),
    'alg': Categorical(*np.arange(0,6))
})

sample_type = 0
if sample_type == 0:
    # Only control
    sample_space = {'init_conditions': init_conditions}
elif sample_type == 1:
    # Only training
    sample_space = {'training_conditions': training_conditions}
else:
    # Control and Training
    sample_space = {'init_conditions': init_conditions, 'training_conditions': training_conditions}

SAMPLERTYPE = 'ce'
MAX_ITERS = 20
PORT = 8888
MAXREQS = 5
BUFSIZE = 4096

if sample_type ==1:
    specification = ["F(~xinrange | ~thetainrange"]
else:
    specification = ["G(xinrange & thetainrange)"]

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.compute_error_table = True
falsifier_params.fal_thres = 0.0

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)

falsifier = mtl_falsifier(sample_space=sample_space, sampler_type=SAMPLERTYPE,
                          specification=specification, falsifier_params=falsifier_params,
                          server_options=server_options)
falsifier.run_falsifier()
analysis_params = DotMap()
analysis_params.k_closest_params.k = 4
analysis_params.random_params.count = 4
falsifier.analyze_error_table(analysis_params=analysis_params)

print("Counter-example samples")
print(falsifier.error_table.table)


# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))
