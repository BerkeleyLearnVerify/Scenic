
"""Interface between Scenic and simulators."""

import enum
import time
import types
from collections import OrderedDict, defaultdict

from scenic.core.object_types import (enableDynamicProxyFor, setDynamicProxyFor,
                                      disableDynamicProxyFor)
from scenic.core.distributions import RejectionException
import scenic.core.dynamics as dynamics
from scenic.core.errors import RuntimeParseError, InvalidScenarioError, optionallyDebugRejection
from scenic.core.requirements import RequirementType
from scenic.core.vectors import Vector

class SimulationCreationError(Exception):
    """Exception indicating a simulation could not be run from the given scene.

    Can also be issued during a simulation if dynamic object creation fails.
    """
    pass

class RejectSimulationException(Exception):
    """Exception indicating a requirement was violated at runtime."""
    pass

class Simulator:
    """A simulator which can execute dynamic simulations from Scenic scenes.

    Simulator interfaces which support dynamic simulations should implement a
    subclass of `Simulator`. An instance of the class represents a connection to
    the simulator suitable for running multiple simulations (not necessarily of
    the same Scenic program). For a simple example of how to implement this class,
    and its counterpart `Simulation` for individual simulations, see
    :mod:`scenic.simulators.lgsvl.simulator`.
    """

    def simulate(self, scene, maxSteps=None, maxIterations=100, verbosity=0,
                 raiseGuardViolations=False):
        """Run a simulation for a given scene.

        Args:
            scene (Scene): Scene from which to start the simulation (sampled using
                `Scenario.generate`).
            maxSteps (int): Maximum number of time steps for the simulation, or `None` to
                not impose a time bound.
            maxIterations (int): Maximum number of rejection sampling iterations.
            verbosity (int): Verbosity level (see :option:`--verbosity`).
            raiseGuardViolations (bool): Whether violations of preconditions/invariants
                of scenarios/behaviors should cause this method to raise an exception,
                instead of only rejecting the simulation (the default behavior).

        Returns:
            A `Simulation` object representing the completed simulation, or `None` if no
            simulation satisfying the requirements could be found within
            **maxIterations** iterations.

        Raises:
            `SimulationCreationError`: if an error occurred while trying to run a
                simulation (e.g. some assumption made by the simulator was violated, like
                trying to create an object inside another).
            `GuardViolation`: if **raiseGuardViolations** is true and a precondition or
                invariant was violated during the simulation.
        """

        # Repeatedly run simulations until we find one satisfying the requirements
        iterations = 0
        while maxIterations is None or iterations < maxIterations:
            iterations += 1
            # Run a single simulation
            try:
                simulation = self.createSimulation(scene, verbosity=verbosity)
                simulation.run(maxSteps)
            except (RejectSimulationException, RejectionException, dynamics.GuardViolation) as e:
                if verbosity >= 2:
                    print(f'  Rejected simulation {iterations} at time step '
                          f'{simulation.currentTime} because: {e}')
                if raiseGuardViolations and isinstance(e, dynamics.GuardViolation):
                    raise
                else:
                    optionallyDebugRejection(e)
                    continue
            # Completed the simulation without violating a requirement
            if verbosity >= 2:
                print(f'  Simulation {iterations} ended successfully at time step '
                      f'{simulation.currentTime} because: {simulation.result.terminationReason}')
            return simulation
        return None

    def createSimulation(self, scene, verbosity=0):
        """Create a `Simulation` from a Scenic scene.

        This should be overridden by subclasses to return instances of their own
        specialized subclass of `Simulation`.
        """
        return Simulation(scene, verbosity=verbosity)

    def destroy(self):
        """Clean up as needed when shutting down the simulator interface."""
        pass

class Simulation:
    """A single simulation run, possibly in progress.

    These objects are not manipulated manually, but are created and executed by a
    `Simulator`. Simulator interfaces should subclass this class, overriding abstract
    methods like `createObjectInSimulator`, `step`, and `getProperties` to call the
    appropriate simulator APIs.

    Attributes:
        result (`SimulationResult`): Result of the simulation, or `None` if it has not
            yet completed. This is the primary object which should be inspected to get
            data out of the simulation: the other attributes of this class are primarily
            for internal use.
    """

    def __init__(self, scene, timestep=1, verbosity=0):
        self.result = None
        self.scene = scene
        self.objects = list(scene.objects)
        self.agents = list(obj for obj in scene.objects if obj.behavior is not None)
        self.trajectory = []
        self.records = defaultdict(list)
        self.currentTime = 0
        self.timestep = timestep
        self.verbosity = verbosity
        self.worker_num = 0

    def run(self, maxSteps):
        """Run the simulation.

        Throws a RejectSimulationException if a requirement is violated.
        """
        trajectory = self.trajectory
        if self.currentTime > 0:
            raise RuntimeError('tried to run a Simulation which has already run')
        assert len(trajectory) == 0
        actionSequence = []

        import scenic.syntax.veneer as veneer
        veneer.beginSimulation(self)
        dynamicScenario = self.scene.dynamicScenario

        try:
            # Initialize dynamic scenario
            dynamicScenario._start()

            # Give objects a chance to do any simulator-specific setup
            for obj in self.objects:
                obj.startDynamicSimulation()

            # Update all objects in case the simulator has adjusted any dynamic
            # properties during setup
            self.updateObjects()

            # Run simulation
            assert self.currentTime == 0
            terminationReason = None
            terminationType = None
            while True:
                if self.verbosity >= 3:
                    print(f'    Time step {self.currentTime}:')

                # Run compose blocks of compositional scenarios
                # (and check if any requirements defined therein fail)
                terminationReason = dynamicScenario._step()
                terminationType = TerminationType.scenarioComplete

                # Record current state of the simulation
                self.recordCurrentState()

                # Run monitors
                newReason = dynamicScenario._runMonitors()
                if newReason is not None:
                    terminationReason = newReason
                    terminationType = TerminationType.terminatedByMonitor

                # "Always" and scenario-level requirements have been checked;
                # now safe to terminate if the top-level scenario has finished,
                # a monitor requested termination, or we've hit the timeout
                if terminationReason is not None:
                    break
                terminationReason = dynamicScenario._checkSimulationTerminationConditions()
                if terminationReason is not None:
                    terminationType = TerminationType.simulationTerminationCondition
                    break
                if maxSteps and self.currentTime >= maxSteps:
                    terminationReason = f'reached time limit ({maxSteps} steps)'
                    terminationType = TerminationType.timeLimit
                    break

                # Compute the actions of the agents in this time step
                allActions = OrderedDict()
                schedule = self.scheduleForAgents()
                for agent in schedule:
                    behavior = agent.behavior
                    if not behavior._runningIterator:   # TODO remove hack
                        behavior._start(agent)
                    actions = behavior._step()
                    if isinstance(actions, EndSimulationAction):
                        terminationReason = str(actions)
                        terminationType = TerminationType.terminatedByBehavior
                        break
                    assert isinstance(actions, tuple)
                    if len(actions) == 1 and isinstance(actions[0], (list, tuple)):
                        actions = tuple(actions[0])
                    if not self.actionsAreCompatible(agent, actions):
                        raise InvalidScenarioError(f'agent {agent} tried incompatible '
                                                   f' action(s) {actions}')
                    allActions[agent] = actions
                if terminationReason is not None:
                    break

                # Execute the actions
                if self.verbosity >= 3:
                    for agent, actions in allActions.items():
                        print(f'      Agent {agent} takes action(s) {actions}')
                actionSequence.append(allActions)
                self.executeActions(allActions)

                # Run the simulation for a single step and read its state back into Scenic
                self.step()
                self.updateObjects()
                self.currentTime += 1

            # Stop all remaining scenarios
            # (and reject if some 'require eventually' condition was never satisfied)
            for scenario in tuple(veneer.runningScenarios):
                scenario._stop('simulation terminated')

            # Record finally-recorded values
            values = dynamicScenario._evaluateRecordedExprs(RequirementType.recordFinal)
            for name, val in values.items():
                self.records[name] = val

            # Package up simulation results into a compact object
            result = SimulationResult(trajectory, actionSequence, terminationType,
                                      terminationReason, self.records)
            self.result = result
            return self
        finally:
            self.destroy()
            for obj in self.scene.objects:
                disableDynamicProxyFor(obj)
            for agent in self.agents:
                if agent.behavior._isRunning:
                    agent.behavior._stop()
            for monitor in self.scene.monitors:
                if monitor._isRunning:
                    monitor._stop()
            # If the simulation was terminated by an exception (including rejections),
            # some scenarios may still be running; we need to clean them up without
            # checking their requirements, which could raise rejection exceptions.
            for scenario in tuple(veneer.runningScenarios):
                scenario._stop('exception', quiet=True)
            veneer.endSimulation(self)

    def createObject(self, obj):
        """Dynamically create an object."""
        if self.verbosity >= 3:
            print(f'      Creating object {obj}')
        self.createObjectInSimulator(obj)
        self.objects.append(obj)
        if obj.behavior:
            self.agents.append(obj)

    def createObjectInSimulator(self, obj):
        """Create the given object in the simulator.

        Implemented by subclasses, and called through `createObject`. Should raise
        SimulationCreationError if creating the object fails.
        """
        raise NotImplementedError

    def recordCurrentState(self):
        dynamicScenario = self.scene.dynamicScenario
        records = self.records

        # Record initially-recorded values
        if self.currentTime == 0:
            values = dynamicScenario._evaluateRecordedExprs(RequirementType.recordInitial)
            for name, val in values.items():
                records[name] = val

        # Record time-series values
        values = dynamicScenario._evaluateRecordedExprs(RequirementType.record)
        for name, val in values.items():
            records[name].append((self.currentTime, val))

        self.trajectory.append(self.currentState())

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
            agent.lastActions = actions

    def step(self):
        """Run the simulation for one step and return the next trajectory element."""
        raise NotImplementedError

    def updateObjects(self):
        """Update the positions and other properties of objects from the simulation."""
        for obj in self.objects:
            # Get latest values of dynamic properties from simulation
            properties = obj._dynamicProperties
            values = self.getProperties(obj, properties)
            assert properties == set(values), properties ^ set(values)

            # Preserve some other properties which are assigned internally by Scenic
            for prop in self.mutableProperties(obj):
                values[prop] = getattr(obj, prop)

            # Make a new copy of the object to ensure that computed properties like
            # visibleRegion, etc. are recomputed
            setDynamicProxyFor(obj, obj.copyWith(**values))

    def mutableProperties(self, obj):
        return {'lastActions', 'behavior'}

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

    @property
    def currentRealTime(self):
        return self.currentTime * self.timestep

    def destroy(self):
        """Perform any cleanup necessary to reset the simulator after a simulation."""
        pass

class DummySimulator(Simulator):
    """Simulator which does nothing, for debugging purposes."""
    def __init__(self, timestep=1):
        self.timestep = timestep

    def createSimulation(self, scene, verbosity=0):
        return DummySimulation(scene, timestep=self.timestep, verbosity=verbosity)

class DummySimulation(Simulation):
    """Minimal `Simulation` subclass for `DummySimulator`."""
    def createObjectInSimulator(self, obj):
        pass

    def actionsAreCompatible(self, agent, actions):
        return True

    def executeActions(self, allActions):
        for agent, actions in allActions.items():
            agent.lastActions = actions

    def step(self):
        pass

    def getProperties(self, obj, properties):
        vals = dict(position=obj.position, heading=obj.heading,
                    velocity=Vector(0, 0), speed=0, angularSpeed=0)
        for prop in properties:
            if prop not in vals:
                vals[prop] = None
        return vals

class Action:
    """An :term:`action` which can be taken by an agent for one step of a simulation."""
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
        return f'"terminate" executed on line {self.line}'

class EndScenarioAction(Action):
    """Special action indicating it is time to end the current scenario.

    Only for internal use.
    """
    def __init__(self, line):
        self.line = line

    def __str__(self):
        return f'"terminate scenario" executed on line {self.line}'

@enum.unique
class TerminationType(enum.Enum):
    """Enum describing the possible ways a simulation can end."""
    #: Simulation reached the specified time limit.
    timeLimit = 'reached simulation time limit'
    #: The top-level scenario's ``compose`` block finished executing.
    scenarioComplete = 'the top-level scenario finished'
    #: A user-specified termination condition was met.
    simulationTerminationCondition = 'a simulation termination condition was met'
    #: A monitor used ``terminate`` to end the simulation.
    terminatedByMonitor = 'a monitor terminated the simulation'
    #: A behavior used ``terminate`` to end the simulation.
    terminatedByBehavior = 'a behavior terminated the simulation'

class SimulationResult:
    """Result of running a simulation.

    Attributes:
        trajectory: A tuple giving for each time step the simulation's 'state': by
            default the positions of every object. See `Simulation.currentState`.
        finalState: The last 'state' of the simulation, as above.
        actions: A tuple giving for each time step a dict specifying for each agent the
            (possibly-empty) tuple of actions it took at that time step.
        terminationType (`TerminationType`): The way the simulation ended.
        terminationReason (str): A human-readable string giving the reason why the
            simulation ended, possibly including debugging info.
        records (dict): For each ``record`` statement, the value or time series of values
            its expression took during the simulation.
    """
    def __init__(self, trajectory, actions, terminationType, terminationReason, records):
        self.trajectory = tuple(trajectory)
        assert self.trajectory
        self.finalState = self.trajectory[-1]
        self.actions = tuple(actions)
        self.terminationType = terminationType
        self.terminationReason = str(terminationReason)
        self.records = dict(records)
