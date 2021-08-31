
"""Interface between Scenic and simulators."""

import enum
import time
import types
from collections import OrderedDict, defaultdict

from scenic.core.object_types import (enableDynamicProxyFor, setDynamicProxyFor,
                                      disableDynamicProxyFor)
from scenic.core.distributions import RejectionException
import scenic.core.dynamics as dynamics
from scenic.core.errors import RuntimeParseError, InvalidScenarioError
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
    """A simulator which can import/execute scenes from Scenic."""

    def simulate(self, scene, maxSteps=None, maxIterations=100, verbosity=0,
                 raiseGuardViolations=False):
        """Run a simulation for a given scene."""

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
                          f'{simulation.currentTime} because of: {e}')
                if raiseGuardViolations and isinstance(e, dynamics.GuardViolation):
                    raise
                else:
                    continue
            # Completed the simulation without violating a requirement
            if verbosity >= 2:
                print(f'  Simulation {iterations} ended successfully at time step '
                      f'{simulation.currentTime} because of: {simulation.result.terminationReason}')
            return simulation
        return None

    def createSimulation(self, scene, verbosity=0):
        return Simulation(scene, verbosity=verbosity)

    def destroy(self):
        pass

class Simulation:
    """A single simulation run, possibly in progress."""

    def __init__(self, scene, timestep=1, verbosity=0):
        self.scene = scene
        self.objects = list(scene.objects)
        self.agents = list(obj for obj in scene.objects if obj.behavior is not None)
        self.trajectory = [self.currentState()]
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
        records = self.records
        if self.currentTime > 0:
            raise RuntimeError('tried to run a Simulation which has already run')
        assert len(trajectory) == 1
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

            # Record initially-recorded values
            values = dynamicScenario._evaluateRecordedExprs(RequirementType.recordInitial)
            for name, val in values.items():
                records[name] = val

            # Run simulation
            assert self.currentTime == 0
            terminationReason = None
            terminationType = None
            while maxSteps is None or self.currentTime < maxSteps:
                if self.verbosity >= 3:
                    print(f'    Time step {self.currentTime}:')

                # Run compose blocks of compositional scenarios
                terminationReason = dynamicScenario._step()
                terminationType = TerminationType.scenarioComplete

                # Record current values of recorded expressions
                values = dynamicScenario._evaluateRecordedExprs(RequirementType.record)
                for name, val in values.items():
                    records[name].append((self.currentTime, val))

                # Check if any requirements fail
                dynamicScenario._checkAlwaysRequirements()

                # Run monitors
                newReason = dynamicScenario._runMonitors()
                if newReason is not None:
                    terminationReason = newReason
                    terminationType = TerminationType.terminatedByMonitor

                # "Always" and scenario-level requirements have been checked;
                # now safe to terminate if the top-level scenario has finished
                # or a monitor requested termination
                if terminationReason is not None:
                    break
                terminationReason = dynamicScenario._checkSimulationTerminationConditions()
                if terminationReason is not None:
                    terminationType = TerminationType.simulationTerminationCondition
                    break

                # Compute the actions of the agents in this time step
                allActions = OrderedDict()
                schedule = self.scheduleForAgents()
                for agent in schedule:
                    behavior = agent.behavior
                    if not behavior._runningIterator:   # TODO remove hack
                        behavior.start(agent)
                    actions = behavior.step()
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
                self.executeActions(allActions)

                # Run the simulation for a single step and read its state back into Scenic
                self.step()
                self.updateObjects()
                self.currentTime += 1

                # Save the new state
                trajectory.append(self.currentState())
                actionSequence.append(allActions)

            # Record finally-recorded values
            values = dynamicScenario._evaluateRecordedExprs(RequirementType.recordFinal)
            for name, val in values.items():
                records[name] = val

            # Package up simulation results into a compact object
            if terminationReason is None:
                terminationReason = f'reached time limit ({maxSteps} steps)'
                terminationType = TerminationType.timeLimit
            result = SimulationResult(trajectory, actionSequence, terminationType,
                                      terminationReason, records)
            self.result = result
            return self
        finally:
            self.destroy()
            for obj in self.scene.objects:
                disableDynamicProxyFor(obj)
            for agent in self.agents:
                agent.behavior.stop()
            for monitor in self.scene.monitors:
                monitor.stop()
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
            assert set(properties) == set(values), set(properties) ^ set(values)

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
        return f'"terminate" on line {self.line}'

class EndScenarioAction(Action):
    """Special action indicating it is time to end the current scenario.

    Only for internal use.
    """
    def __init__(self, line):
        self.line = line

    def __str__(self):
        return f'"terminate scenario" on line {self.line}'

@enum.unique
class TerminationType(enum.Enum):
    timeLimit = 'reached simulation time limit'
    scenarioComplete = 'the top-level scenario finished'
    simulationTerminationCondition = 'a simulation termination condition was met'
    terminatedByMonitor = 'a monitor terminated the simulation'
    terminatedByBehavior = 'a behavior terminated the simulation'

class SimulationResult:
    """Result of running a simulation."""
    def __init__(self, trajectory, actions, terminationType, terminationReason, records):
        self.trajectory = tuple(trajectory)
        assert self.trajectory
        self.finalState = self.trajectory[-1]
        self.actions = tuple(actions)
        self.terminationType = terminationType
        self.terminationReason = str(terminationReason)
        self.records = dict(records)
