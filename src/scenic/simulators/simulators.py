
"""Interface between Scenic and simulators."""

import types
from collections import OrderedDict

from scenic.core.object_types import enableDynamicProxyFor, disableDynamicProxyFor
from scenic.core.utils import RuntimeParseError

class SimulationCreationError(Exception):
    """Exception indicating a simulation could not be run from the given scene."""
    pass

class RejectSimulationException(Exception):
    """Exception indicating a requirement was violated at runtime."""
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
                trajectory, terminationReason = simulation.run(maxSteps, verbosity=verbosity)
            except RejectSimulationException as e:
                if verbosity >= 2:
                    print(f'  Rejected simulation {iterations} at time step '
                          f'{simulation.currentTime} because of: {e}')
                continue
            # Completed the simulation without violating a requirement
            if verbosity >= 2:
                print(f'  Simulation {iterations} ended successfully at time step '
                      f'{simulation.currentTime} because of: {terminationReason}')
            return trajectory, simulation
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

    def run(self, maxSteps, verbosity=0):
        """Run the simulation.

        Throws a RejectSimulationException if a requirement is violated.
        """
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
                running = agent.behavior.start(agent)
                if not running:
                    raise RuntimeError(f'{agent.behavior} of {agent} does not take any actions')
            # Initialize monitor coroutines
            for monitor in self.scene.monitors:
                monitor.start()

            # Run simulation
            assert self.currentTime == 0
            terminationReason = None
            while maxSteps is None or self.currentTime < maxSteps:
                if verbosity >= 3:
                    print(f'    Time step {self.currentTime}:')

                # Check if any requirements fail
                for req in self.scene.alwaysRequirements:
                    if not req.isTrue():
                        # always requirements should never be violated at time 0, since
                        # they are enforced during scene sampling
                        assert self.currentTime > 0
                        raise RejectSimulationException(str(req))

                # Run monitors
                for monitor in self.scene.monitors:
                    action = monitor.step()
                    if isinstance(action, EndSimulationAction):
                        terminationReason = str(action)

                # Compute the actions of the agents in this time step
                actions = OrderedDict()
                schedule = self.scheduleForAgents()
                for agent in schedule:
                    action = agent.behavior.step()
                    if isinstance(action, EndSimulationAction):
                        terminationReason = str(action)
                    actions[agent] = action
                    if verbosity >= 3:
                        print(f'      Agent {agent} takes action {action}')

                # All requirements have been checked; now safe to terminate if requested by
                # a behavior/monitor or if a termination condition is met
                for req in self.scene.terminationConditions:
                    if req.isTrue():
                        terminationReason = str(req)
                if terminationReason is not None:
                    break

                # Run the simulation for a single step
                nextState = self.step(actions)
                trajectory.append(nextState)
                self.currentTime += 1

            if terminationReason is None:
                terminationReason = f'reached time limit ({maxSteps} steps)'
            return trajectory, terminationReason
        finally:
            for obj in self.scene.objects:
                disableDynamicProxyFor(obj)
            for agent in self.agents:
                agent.behavior.stop()
            for monitor in self.scene.monitors:
                monitor.stop()
            veneer.endSimulation(self)

    def initialState(self):
        """Return the initial state of the simulation."""
        return None

    def scheduleForAgents(self):
        """Return the order for the agents to run in the next time step."""
        return self.agents

    def step(self, actions):
        """Run the simulation for a step, given the actions of the agents.

        Note that actions is an OrderedDict, as the order of actions may matter."""
        return tuple(actions.values())

class Action:
    """An action which can be taken by an agent for one step of a simulation."""
    pass

class EndSimulationAction(Action):
    """Special action indicating it is time to end the simulation.

    Only for internal use.
    """
    def __init__(self, line):
        self.line = line

    def __str__(self):
        return f'"terminate" on line {self.line}'
