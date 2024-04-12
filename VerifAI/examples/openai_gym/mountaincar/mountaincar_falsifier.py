from verifai.features.features import *
from verifai.falsifier import generic_falsifier
from dotmap import DotMap
from verifai.monitor import specification_monitor

try:
    import gym
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires OpenAI-gym to be installed')

env_id = 'MountainCar-v0'
mc = gym.make(env_id)

init_conditions = Struct({
        'x_init': Box([-0.6, -0.4]),
        'v_init': Box([-0.25, 0.25]),
        'goal_position':Box([mc.env.goal_position - 0.1, mc.env.goal_position+0.1])
    })

training_conditions = Struct({
    'num_timesteps': DiscreteBox([10000, 100000]),
    'alg': Categorical(*np.arange(6))
})

sample_type = 1
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

class mountaincar_spec(specification_monitor):
    def __init__(self):
        def specification(traj):
            traj_x = traj['traj_x']
            if sample_type == 1:
                return mc.env.goal_position - max(traj_x)
            else:
                return max(traj_x) - mc.env.goal_position
        super().__init__(specification)

falsifier_params = DotMap()
falsifier_params.n_iters = MAX_ITERS
falsifier_params.compute_error_table = True
falsifier_params.fal_thres = 0.0

server_options = DotMap(port=PORT, bufsize=BUFSIZE, maxreqs=MAXREQS)


falsifier = generic_falsifier(sample_space=sample_space, sampler_type=SAMPLERTYPE,
                              monitor=mountaincar_spec(), falsifier_params=falsifier_params,
                              server_options=server_options)
falsifier.run_falsifier()
analysis_params = DotMap()
analysis_params.k_closest_params.k = 4
analysis_params.random_params.count = 4
falsifier.analyze_error_table(analysis_params=analysis_params)

print("Hyper-parameters leading to good controllers")
print(falsifier.error_table.table)


# To save all samples: uncomment this
# pickle.dump(falsifier.samples, open("generated_samples.pickle", "wb"))
