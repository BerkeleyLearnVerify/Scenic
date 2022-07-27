from scenic.core.simulators import Action


class NNAction(Action):
    def __init__(self):
        # Load model here and initialize parameters
        self.model = None
        pass

    def input_processing(self, observation):
        # Do conversion from simulator data type to NN input type and shape
        return observation

    def output_processing(self, action):
        # Do conversion from NN output type and shape to simulator action
        return action

    def applyTo(self, agent, simulation):
        observation = agent.observations["front_rgb"]
        action = self.model(self.input_processing(observation))
        agent.set_action(self.output_processing(action))
