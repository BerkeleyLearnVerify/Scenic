
"""Interface between Scenic and simulators."""

import types
from collections import OrderedDict

from scenic.core.object_types import (enableDynamicProxyFor, setDynamicProxyFor,
                                      disableDynamicProxyFor)
from scenic.core.distributions import RejectionException
from scenic.core.errors import RuntimeParseError, InvalidScenarioError
from scenic.core.vectors import Vector

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
                result = simulation.run(maxSteps, verbosity=verbosity)
            except (RejectSimulationException, RejectionException) as e:
                if verbosity >= 2:
                    print(f'  Rejected simulation {iterations} at time step '
                          f'{simulation.currentTime} because of: {e}')
                continue
            # Completed the simulation without violating a requirement
            if verbosity >= 2:
                print(f'  Simulation {iterations} ended successfully at time step '
                      f'{simulation.currentTime} because of: {result.terminationReason}')
            return result
        return None

    def createSimulation(self, scene):
        return Simulation(scene)

class Simulation:
    """A single simulation run, possibly in progress."""

    def __init__(self, scene, timestep=1):
        self.scene = scene
        self.objects = scene.objects
        self.agents = tuple(obj for obj in scene.objects if obj.behavior is not None)
        self.trajectory = [self.currentState()]
        self.currentTime = 0
        self.timestep = timestep

    def run(self, maxSteps, verbosity=0):
        """Run the simulation.

        Throws a RejectSimulationException if a requirement is violated.
        """
        global runningSimulation

        trajectory = self.trajectory
        if self.currentTime > 0:
            raise RuntimeError('tried to run a Simulation which has already run')
        assert len(trajectory) == 1
        actionSequence = []

        import scenic.syntax.veneer as veneer
        veneer.beginSimulation(self)

        try:
            # Initialize behavior coroutines of agents
            for agent in self.agents:
                running = agent.behavior.start(agent)
                if not running:
                    raise RuntimeError(f'{agent.behavior} of {agent} does not take any actions')
            # Initialize monitor coroutines
            for monitor in self.scene.monitors:
                monitor.start()

            # Update all objects in case the simulator has adjusted any dynamic
            # properties during setup
            self.updateObjects()

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
                        break
                if terminationReason is not None:
                    break

                # Compute the actions of the agents in this time step
                allActions = OrderedDict()
                schedule = self.scheduleForAgents()
                for agent in schedule:
                    actions = agent.behavior.step()
                    if isinstance(actions, EndSimulationAction):
                        terminationReason = str(actions)
                        break
                    assert isinstance(actions, tuple)
                    actions = tuple(a for a in actions if a is not None)
                    if not self.actionsAreCompatible(agent, actions):
                        raise InvalidScenarioError(f'agent {agent} tried incompatible '
                                                   f' action(s) {actions}')
                    allActions[agent] = actions
                    if verbosity >= 3:
                        print(f'      Agent {agent} takes action(s) {actions}')
                if terminationReason is not None:
                    break

                # All requirements have been checked; now safe to terminate if requested by
                # a behavior/monitor or if a termination condition is met
                for req in self.scene.terminationConditions:
                    if req.isTrue():
                        terminationReason = str(req)
                        break
                if terminationReason is not None:
                    break

                # Execute the actions
                self.executeActions(allActions)

                # Run the simulation for a single step and read its state back into Scenic
                self.step()
                self.updateObjects()
                self.currentTime += 1

                # Save the new state
                trajectory.append(self.currentState())
                actionSequence.append(allActions)

            if terminationReason is None:
                terminationReason = f'reached time limit ({maxSteps} steps)'
            result = SimulationResult(trajectory, actionSequence, terminationReason)
            return result
        finally:
            for obj in self.scene.objects:
                disableDynamicProxyFor(obj)
            for agent in self.agents:
                agent.behavior.stop()
            for monitor in self.scene.monitors:
                monitor.stop()
            veneer.endSimulation(self)

    def scheduleForAgents(self):
        """Return the order for the agents to run in the next time step."""
        return self.agents

    def actionsAreCompatible(self, agent, actions):
        """Check whether the given actions can be taken simultaneously by an agent.

        The default is to have all actions compatible with each other and all agents.
        Subclasses should override this method as appropriate.
        """
        for action in actions:
            if not action.canBeTakenBy(agent):
                return False
        return True

    def executeActions(self, allActions):
        """Execute the actions selected by the agents.

        Note that ``allActions`` is an OrderedDict, as the order of actions may matter.
        """
        for agent, actions in allActions.items():
            for action in actions:
                action.applyTo(agent, self)

    def step(self):
        """Run the simulation for one step and return the next trajectory element."""
        raise NotImplementedError

    def updateObjects(self):
        """Update the positions and other properties of objects from the simulation."""
        for obj in self.objects:
            properties = obj._dynamicProperties
            values = self.getProperties(obj, properties)
            assert set(properties) == set(values), set(properties) ^ set(values)

            # Make a new copy of the object to ensure that computed properties like
            # visibleRegion, etc. are recomputed
            setDynamicProxyFor(obj, obj.copyWith(**values))

    def getProperties(self, obj, properties):
        """Read the values of the given properties of the object from the simulation."""
        raise NotImplementedError

    def currentState(self):
        """Return the current state of the simulation.

        The definition of 'state' is up to the simulator; the 'state' is simply saved
        at each time step to define the 'trajectory' of the simulation.

        The default implementation returns a tuple of the positions of all objects.
        """
        return tuple(obj.position for obj in self.objects)

class DummySimulator(Simulator):
    """Simulator which does nothing, for debugging purposes."""
    def createSimulation(self, scene):
        return DummySimulation(scene)

class DummySimulation(Simulation):
    def actionsAreCompatible(self, agent, actions):
        return True

    def executeActions(self, actions):
        pass

    def step(self):
        pass

    def getProperties(self, obj, properties):
        return dict(position=obj.position, heading=obj.heading,
                    velocity=Vector(0, 0), speed=0, angularSpeed=0)

class Action:
    """An action which can be taken by an agent for one step of a simulation."""
    def canBeTakenBy(self, agent):
        return True

    def applyTo(self, agent, simulation):
        raise NotImplementedError

class EndSimulationAction(Action):
    """Special action indicating it is time to end the simulation.

    Only for internal use.
    """
    def __init__(self, line):
        self.line = line

    def __str__(self):
        return f'"terminate" on line {self.line}'

class SimulationResult:
    """Result of running a simulation."""
    def __init__(self, trajectory, actions, terminationReason):
        self.trajectory = tuple(trajectory)
        assert self.trajectory
        self.finalState = self.trajectory[-1]
        self.actions = tuple(actions)
        self.terminationReason = terminationReason
