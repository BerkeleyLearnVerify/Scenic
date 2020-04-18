
"""Interface between Scenic and simulators."""

import types
from collections import OrderedDict

from scenic.core.object_types import enableDynamicProxyFor, disableDynamicProxyFor
from scenic.core.utils import RuntimeParseError

class RejectSimulationException(Exception):
    """Exception indicating a requirement was violated at runtime."""
    pass

class EndSimulationException(Exception):
    """Exception indicating it is time to end the simulation."""
    pass

class Simulator:
    """A simulator which can import/execute scenes from Scenic."""

    def simulate(self, scene, maxSteps=None, maxIterations=100, verbosity=0):
        """Run a simulation for a given scene."""

        # Repeatedly run simulations until we find one satisfying the requirements
        iterations = 0
        while maxIterations is None or iterations < maxIterations:
            iterations += 1
            # Run a single simulation
            try:
                simulation = self.createSimulation(scene)
                trajectory = simulation.run(maxSteps)
            except RejectSimulationException as e:
                if verbosity >= 2:
                    print(f'  Rejected simulation {iterations} at time step '
                          f'{simulation.currentTime} because of: {e}')
                continue
            # Completed the simulation without violating a requirement
            return trajectory
        raise RuntimeError(f'failed to generate valid simulation in {maxIterations} iterations')

    def createSimulation(self, scene):
        return Simulation(scene)

class Simulation:
    """A single simulation run, possibly in progress."""

    def __init__(self, scene):
        self.scene = scene
        self.objects = scene.objects
        self.agents = tuple(obj for obj in scene.objects if obj.behavior is not None)
        self.trajectory = [self.initialState()]
        self.currentTime = 0

    def run(self, maxSteps):
        """Run the simulation.

        Throws a RejectSimulationException if a requirement is violated."""
        global runningSimulation

        trajectory = self.trajectory
        if self.currentTime > 0:
            raise RuntimeError('tried to run a Simulation which has already run')
        assert len(trajectory) == 1

        import scenic.syntax.veneer as veneer
        veneer.beginSimulation(self)

        # Set up mutable proxies for all objects
        for obj in self.scene.objects:
            enableDynamicProxyFor(obj)

        try:
            # Initialize behavior coroutines of agents
            for agent in self.agents:
                agent.behavior = agent.behavior(agent)
                if not isinstance(agent.behavior, types.GeneratorType):
                    raise RuntimeParseError(f'behavior of {agent} does not invoke any actions')

            # Run simulation
            assert self.currentTime == 0
            while maxSteps is None or self.currentTime < maxSteps:
                # Check if any requirements fail or termination conditions hold
                for req in self.scene.alwaysRequirements:
                    if not req.isTrue():
                        # always requirements should never be violated at time 0, since
                        # they are enforced during scene sampling
                        assert self.currentTime > 0
                        raise RejectSimulationException(f'always requirement (line {req.line})')
                terminated = False
                for req in self.scene.terminationConditions:
                    if req.isTrue():
                        terminated = True
                        break
                if terminated:
                    break
                # Compute the actions of the agents in this time step
                actions = OrderedDict()
                schedule = self.scheduleForAgents()
                for agent in schedule:
                    try:
                        action = agent.behavior.send(None)
                    except StopIteration as e:
                        action = None      # behavior ended early; take no action
                    except EndSimulationException:
                        terminated = True
                        break
                    actions[agent] = action
                if terminated:
                    break
                # Run the simulation for a single step
                nextState = self.step(actions)
                trajectory.append(nextState)
                self.currentTime += 1
            return trajectory
        finally:
            for obj in self.scene.objects:
                disableDynamicProxyFor(obj)
            veneer.endSimulation()

    def initialState(self):
        """Return the initial state of the simulation."""
        return None

    def scheduleForAgents(self):
        """Return the order for the agents to run in the next time step."""
        return self.agents

    def step(self, actions):
        """Run the simulation for a step, given the actions of the agents.

        Note that actions is an OrderedDict, as the order of actions may matter."""
        #for agent, action in actions.items():
        #    print(f'agent {agent} takes action {action}')
        return tuple(actions.values())

class Action:
    """An action which can be taken by an agent for one step of a simulation."""
    pass
