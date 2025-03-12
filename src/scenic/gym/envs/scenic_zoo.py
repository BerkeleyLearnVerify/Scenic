from scenic.core.simulators import Simulator, Simulation
from pettingzoo import ParllelEnv 

class ScenicGymEnv(ParallelEnv):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4} # TODO placeholder, add simulator-specific entries
    # TODO determine where to pass in reward function
    def __init__(self, 
                 render_mode=None, 
                 scenic_program : str = None, # TODO add code to allow both directory and program itself
                 simulator : scenic.core.simulators.Simulator = None, 
                 verifai_sampler="scenic", 
                 max_steps = 1000
                 observation_space : spaces.Dict = spaces.Dict(),
                 action_space : spaces.Dict = spaces.Dict()): # empty string means just pure scenic???

        self.observation_space = observation_space
        self.action_space = action_space

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.max_steps = max_steps
        self.simulator = simulator
        self.scene = self._make_run_loop() # can do this first, since simulation won't step until we call next(self.scene)

    def _make_run_loop(self):
        while True:
            try:
                with simulator.simulateStepped(self.scene, maxSteps=self.max_steps) as simulation:
                    # this first block before the while loop is for the first reset call
                    done = lambda: not (simulation.result is None)

                    simulation.advance()
                    observation = self._get_obs()
                    info = self._get_info()
                    action = yield observation, info
                    simulation.action_dict = action # TODO add action dict to simulation interfaces

                    while not done():
                        # Probably good that we advance first before any action is set.
                        # this is consistent with how reset works
                        simulation.advance()
                        observation = self._get_obs()
                        info = self._get_info()
                        action = yield observation, info, done(), info

                        if done():
                            simulation.destroy()

                        simulation.action_dict = action # TODO add action dict to simulation interfaces
                        
                        # TODO add some logic with regards to rendering

    def _get_obs(self):
        return self.scene.getObs() # TODO add this function in simulator interfaces
    
    def _get_info(self):
        return self.scene.getInfo()

    def reset(self, seed=None, options=None): # TODO will setting seed here conflict with VerifAI's setting of seed?
        # only setting enviornment seed, not torch seed?
        super().reset(seed=seed)
        self.scene.throw(ResetException())
        observation, info = next(self.scene) # not doing self.scene.send(action) just yet

        return observation, info
        
    def step(self, action):
        observation, reward, done, info = self.scene.send(action)

    def render(): # TODO figure out if this function has to be implemented here or if super() has default implementation
        pass

    def close():
        self.simulator.destroy()
    
    def action_space(self, agent):
        return self.observation_space[agent]

    def observation_space(self, agent):
        return self.action_spaces[agent]

