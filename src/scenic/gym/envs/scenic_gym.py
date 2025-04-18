from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
import gymnasium as gym
from gymnasium import spaces
from typing import Callable

#TODO make ResetException
class ResetException(Exception):
    def __init__(self):
        super().__init__("Resetting")

class ScenicGymEnv(gym.Env):
    """
    verifai_sampler now not an argument added in here, but one specified int he Scenic program
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4} # TODO placeholder, add simulator-specific entries
    
    def __init__(self, 
                 scenario : Scenario,
                 simulator : Simulator,
                 render_mode=None, 
                 max_steps = 1000,
                 observation_space : spaces.Dict = spaces.Dict(),
                 action_space : spaces.Dict = spaces.Dict()): # empty string means just pure scenic???

        assert render_mode is None or render_mode in self.metadata["render_modes"]

        self.observation_space = observation_space
        self.action_space = action_space
        self.render_mode = render_mode
        self.max_steps = max_steps
        self.simulator = simulator
        self.scenario = scenario
        self.simulation_results = []

        self.feedback_result = None
        self.loop = None

    def _make_run_loop(self):

        while True:
            try:
                scene, _ = self.scenario.generate(feedback=self.feedback_result)
                with self.simulator.simulateStepped(scene, maxSteps=self.max_steps) as simulation:
                    steps_taken = 0
                    # this first block before the while loop is for the first reset call
                    done = lambda: not (simulation.result is None)
                    truncated = lambda: (steps_taken >= self.max_steps) # TODO handle cases where it is done right on maxsteps
                    observation = simulation.get_obs()
                    info = simulation.get_info() 
                    actions = yield observation, info
                    simulation.actions = actions # TODO add action dict to simulation interfaces

                    while not done():
                        # Probably good that we advance first before any action is set.
                        # this is consistent with how reset works
                        simulation.advance()
                        steps_taken += 1
                        observation = simulation.get_obs()
                        info = simulation.get_info()
                        reward = simulation.get_reward()

                        if done():
                            self.feedback_result = simulation.result
                            self.simulation_results.append(simulation.result)
                            simulation.destroy()
                            actions = yield observation, reward, done(), truncated(), info
                            break # a little unclean right here

                        actions = yield observation, reward, done(), truncated(), info
                        simulation.actions = actions # TODO add action dict to simulation interfaces
                        
            except ResetException:
                continue

    def reset(self, seed=None, options=None): # TODO will setting seed here conflict with VerifAI's setting of seed?
        # only setting enviornment seed, not torch seed?
        super().reset(seed=seed)
        if self.loop is None:
            self.loop = self._make_run_loop()
            observation, info = next(self.loop) # not doing self.scene.send(action) just yet
        else:
            observation, info = self.loop.throw(ResetException())


        return observation, info
        
    def step(self, action):
        assert not (self.loop is None), "self.loop is None, have you called reset()?"

        observation, reward, terminated, truncated, info = self.loop.send(action)
        return observation, reward, terminated, truncated, info

    def render(self): # TODO figure out if this function has to be implemented here or if super() has default implementation
        """
        likely just going to be something like simulation.render() or something
        """
        # FIXME for one project only...also a bit hacky...
        # self.env.render()
        pass

    def close(self):
        self.simulator.destroy()

