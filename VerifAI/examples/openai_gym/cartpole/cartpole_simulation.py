from verifai.simulators.openai_gym.baselines_task import *
from verifai.simulators.openai_gym.client_gym import *
from dotmap import DotMap
import numpy as np

# 0 for control only, 1 for training only and >=2 for both
sample_type = 0
class cartpole_standing(control_task):
    def __init__(self, baselines_params=None):
        if baselines_params is None:
            baselines_params = DotMap()
            baselines_params.alg = 'ppo2'
            baselines_params.env_id = 'CartPole-v1'
            baseline_params.num_timesteps = 1e5
        else:
            if 'env_id' not in baseline_params or baseline_params.env_id !='CartPole-v1':
                baseline_params.env_id = 'CartPole-v1'
            if 'alg' not in baseline_params:
                baseline_params.alg = 'ppo2'
        super().__init__(baselines_params=baselines_params)
        if sample_type >= 1:
            self.run_task = self.run_task_retrain
        self.algs = ['ppo2', 'deepq', 'acer', 'a2c', 'trpo_mpi', 'acktr']


    def use_sample(self, sample):
        if sample_type == 0 or sample_type >=2:
            init_condition = sample.init_conditions
            x_init, theta_init, length, masscart,masspole = init_condition.x_init[0], \
                                         init_condition.theta_init[0], \
                                         init_condition.length[0], \
                                         init_condition.masscart[0], \
                                         init_condition.masspole[0]
            self.init_state = np.array([x_init, 0.0, theta_init, 0.0])
            self.env.env.length = length
            self.env.env.masscart = masscart
            self.env.env.masspole = masspole
        if sample_type >=1:
            training_condition = sample.training_conditions
            num_timesteps, alg = int(training_condition.num_timesteps[0]), \
                                 training_condition.alg
            self.num_timesteps = num_timesteps
            self.alg = self.algs[alg]
            self.train()
        if self.model is None:
            self.train()

    def trajectory_definition(self):
        traj = np.array(self.trajectory)
        traj_x, _, traj_theta, _ = traj.T
        xinrange = [(j*self.env.env.tau, self.env.env.x_threshold + 0.1 - np.abs(v))
                    for j,v in enumerate(traj_x)]
        thetainrange = [(j*self.env.env.tau, self.env.env.theta_threshold_radians + 0.01 - np.abs(v))
                        for j, v in enumerate(traj_theta)]
        sim_results= {'xinrange':xinrange, 'thetainrange':thetainrange}
        return sim_results


PORT = 8888
BUFSIZE = 4096
N_SIM_STEPS = 500
simulation_data = DotMap()
simulation_data.port = PORT
simulation_data.bufsize = BUFSIZE
baseline_params = DotMap()
baseline_params.num_timesteps = 1e5
simulation_data.task = cartpole_standing(baselines_params=baseline_params)

client_task = ClientGym(simulation_data)
while True:
    if not client_task.run_client():
        print("End of all cartpole simulations")
        break
