
import numpy as np

try:
    import gym
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires OpenAI-gym to be installed')

try:
    from baselines.run import *
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires baselines to be installed: Try installing from source')

try:
    import tensorflow as tf
except ModuleNotFoundError:
    import sys
    sys.exit('This functionality requires tensorflow to be installed')




class control_task:
    def __init__(self, baselines_params):
        self.env_id = baselines_params.env_id \
            if 'env_id' in baselines_params else 'CartPole-v1'
        self.alg = baselines_params.alg \
            if 'alg' in baselines_params else 'ppo2'
        self.num_timesteps = baselines_params.num_timesteps \
            if 'num_timesteps' in baselines_params else 1e6
        self.seed = baselines_params.seed \
                if 'seed' in baselines_params else None
        self.save_path = baselines_params.save_path \
                if 'save_path' is baselines_params else None
        self.network = baselines_params.network \
                if 'network' is baselines_params else None

        self.time_horizon = baselines_params.time_horizon \
            if 'time_horizon' in baselines_params else None

        self.env = gym.make(id=self.env_id)
        self.env.reset()
        self.model = None
        self.init_state = np.zeros(self.env.observation_space.shape)
        self.run_task = self.run_task_wo_retrain

    def build_args(self):
        self.args = ['', '--env=' + str(self.env_id), '--alg=' + str(self.alg), \
                     '--num_timesteps=' + str(self.num_timesteps)]
        if self.seed is not None:
            self.args.append('--seed=' + str(self.seed))
        if self.save_path is not None:
            self.args.append('--seed=' + str(self.save_path))
        if self.network is not None:
            self.args.append('--network='+str(self.network))

    def train(self):
        print("Training ", self.env_id, " with ", self.alg, " with ", self.num_timesteps)
        self.build_args()
        self.model = main(self.args)


    def play(self):
        state = self.model.initial_state if hasattr(self.model, 'initial_state') else None
        dones = np.zeros((1,))
        self.env.env.state = self.init_state
        obs = self.env.env.state
        self.trajectory = [obs]
        print("Initial state ", self.init_state)
        i = 0
        while True:
            i += 1
            obs_a = np.array(obs).reshape(1, len(obs))
            if state is not None:
                actions, _, state, _ = self.model.step(obs_a,S=state, M=dones)
            else:
                actions, _, _, _ = self.model.step(obs_a)
            obs, _, done, _ = self.env.step(actions[0])
            self.trajectory.append(obs)
            self.env.render()
            done = self.stop_simulation(done, i)
            if done:
                break
        self.env.close()

    def stop_simulation(self, done, i):
        if self.time_horizon is None:
            return done.any() if isinstance(done, np.ndarray) else done
        else:
            return i == self.time_horizon

    def use_sample(self, sample):
        raise NotImplementedError("Define in child task")

    def trajectory_definition(self):
        raise NotImplementedError("Define in child task")

    def run_task_retrain(self, sample):
        with tf.Session() as sess:
            self.env.reset()
            self.use_sample(sample)
            self.play()
            sess.close()
        tf.reset_default_graph()
        return self.trajectory_definition()

    def run_task_wo_retrain(self, sample):
        self.env.reset()
        self.use_sample(sample)
        self.play()
        return self.trajectory_definition()