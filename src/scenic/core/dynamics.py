"""Support for dynamic behaviors and modular scenarios."""

from collections import defaultdict
import enum
import inspect
import itertools
import types

from scenic.core.distributions import Samplable, toDistribution, needsSampling
from scenic.core.errors import RuntimeParseError, InvalidScenarioError
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.requirements import (RequirementType, PendingRequirement,
                                      DynamicRequirement)
from scenic.core.simulators import (RejectSimulationException, EndSimulationAction,
                                    EndScenarioAction)
from scenic.core.utils import argsToString
from scenic.core.workspaces import Workspace

# Scenarios

class Invocable:
    def __init__(self, *args, **kwargs):
        if veneer.evaluatingGuard:
            raise RuntimeParseError(
                'tried to invoke behavior/scenario from inside guard or interrupt condition')
        self._runningIterator = None

    def _invokeSubBehavior(self, agent, subs, modifier=None):
        if modifier:
            if modifier.name == 'for':  # do X for Y [seconds | steps]
                timeLimit = modifier.value
                if not isinstance(timeLimit, (float, int)):
                    raise RuntimeParseError('"do X for Y" with Y not a number')
                assert modifier.terminator in (None, 'seconds', 'steps')
                if modifier.terminator != 'steps':
                    timeLimit /= veneer.currentSimulation.timestep
                startTime = veneer.currentSimulation.currentTime
                condition = lambda: veneer.currentSimulation.currentTime - startTime >= timeLimit
            elif modifier.name == 'until':  # do X until Y
                condition = modifier.value
            else:
                raise RuntimeError(f'internal parsing error: impossible modifier {modifier}')

            def body(behavior, agent):
                yield from self._invokeInner(agent, subs)
            handler = lambda behavior, agent: BlockConclusion.ABORT
            yield from runTryInterrupt(self, agent, body, [condition], [handler])
        else:
            yield from self._invokeInner(agent, subs)

    def _invokeInner(self, agent, subs):
        raise NotImplementedError

class DynamicScenario(Invocable):
    def __init_subclass__(cls, *args, **kwargs):
        veneer.registerDynamicScenarioClass(cls)

    _requirementSyntax = None   # overridden by subclasses
    _simulatorFactory = None
    _globalParameters = None

    def __init__(self, *args, **kwargs):
        super().__init__()
        self._args = args
        self._kwargs = kwargs

        self._ego = None
        self._objects = []      # ordered for reproducibility
        self._externalParameters = []
        self._pendingRequirements = defaultdict(list)
        self._requirements = []
        self._requirementDeps = set()   # things needing to be sampled to evaluate the requirements

        self._agents = []
        self._monitors = []
        self._behaviors = []
        self._alwaysRequirements = []
        self._terminationConditions = []
        self._terminateSimulationConditions = []
        self._recordedExprs = []
        self._recordedInitialExprs = []
        self._recordedFinalExprs = []

        self._subScenarios = []
        self._endWithBehaviors = False
        self._timeLimit = None
        self._timeLimitIsInSeconds = False
        self._prepared = False
        self._dummyNamespace = None

        self._timeLimitInSteps = None   # computed at simulation time
        self._elapsedTime = 0

    @classmethod
    def _dummy(cls, filename, namespace):
        scenario = cls()
        scenario._setup = None
        scenario._compose = None
        scenario._filename = filename   # for debugging
        scenario._prepared = True
        scenario._dummyNamespace = namespace
        return scenario

    @classmethod
    def _requiresArguments(cls):
        """Whether this scenario cannot be instantiated without arguments."""
        if cls._setup:
            func = cls._setup
        elif cls._compose:
            func = cls._compose
        else:
            return True
        sig = inspect.signature(func)
        try:
            sig.bind(None, None)    # first two arguments are added internally by Scenic
            return False
        except TypeError:
            return True

    @property
    def ego(self):
        return self._ego

    @property
    def objects(self):
        return tuple(self._objects)

    def _bindTo(self, scene):
        self._ego = scene.egoObject
        self._objects = list(scene.objects)
        self._agents = [obj for obj in scene.objects if obj.behavior is not None]
        self._alwaysRequirements = scene.alwaysRequirements
        self._terminationConditions = scene.terminationConditions
        self._terminateSimulationConditions = scene.terminateSimulationConditions
        self._recordedExprs = scene.recordedExprs
        self._recordedInitialExprs = scene.recordedInitialExprs
        self._recordedFinalExprs = scene.recordedFinalExprs

    def _prepare(self):
        assert not self._prepared
        self._prepared = True

        veneer.prepareScenario(self)
        with veneer.executeInScenario(self, inheritEgo=True):
            # Execute setup block
            if self._setup is not None:
                locs = self._setup(self, None, *self._args, **self._kwargs)
                self.__dict__.update(locs)  # save locals as if they were in class scope
                veneer.finishScenarioSetup(self)

        # Extract requirements, scan for relations used for pruning, and create closures
        self._compileRequirements()

    @classmethod
    def _bindGlobals(cls, globs):
        cls._globalParameters = globs

    def _start(self):
        assert self._prepared

        # Compute time limit now that we know the simulation timestep
        self._elapsedTime = 0
        self._timeLimitInSteps = self._timeLimit
        if self._timeLimitIsInSeconds:
            self._timeLimitInSteps /= veneer.currentSimulation.timestep

        veneer.startScenario(self)
        with veneer.executeInScenario(self):
            # Start compose block
            if self._compose is not None:
                if not inspect.isgeneratorfunction(self._compose):
                    from scenic.syntax.translator import composeBlock
                    raise RuntimeParseError(f'"{composeBlock}" does not invoke any scenarios')
                self._runningIterator = self._compose()

            # Initialize behavior coroutines of agents
            for agent in self._agents:
                assert isinstance(agent.behavior, Behavior), agent.behavior
                agent.behavior.start(agent)
            # Initialize monitor coroutines
            for monitor in self._monitors:
                monitor.start()

    def _step(self):
        # Check if we have reached the time limit, if any
        if self._timeLimitInSteps is not None and self._elapsedTime >= self._timeLimitInSteps:
            return self._stop('reached time limit')
        self._elapsedTime += 1

        # Execute compose block, if any
        composeDone = False
        if self._runningIterator is None:
            composeDone = True      # compose block ended in an earlier step
        else:
            with veneer.executeInScenario(self):
                try:
                    result = self._runningIterator.send(None)
                    if isinstance(result, (EndSimulationAction, EndScenarioAction)):
                        return self._stop(result)
                except StopIteration:
                    self._runningIterator = None
                    composeDone = True

        # If there is a compose block and it has finished, we're done
        if self._compose is not None and composeDone:
            return self._stop('finished compose block')

        # Optionally end when all our agents' behaviors have ended
        if self._endWithBehaviors:
            if all(agent.behavior._isFinished for agent in self._agents):
                return self._stop('all behaviors finished')

        # Check if any termination conditions apply
        for req in self._terminationConditions:
            if req.isTrue():
                return self._stop(req)

        # Scenario will not terminate yet
        return None

    def _stop(self, reason):
        veneer.endScenario(self, reason)
        self._runningIterator = None
        return reason

    def _invokeInner(self, agent, subs):
        for sub in subs:
            if not isinstance(sub, DynamicScenario):
                raise RuntimeParseError(f'expected a scenario, got {sub}')
            sub._prepare()
            sub._start()
        self._subScenarios = list(subs)
        while True:
            newSubs = []
            for sub in self._subScenarios:
                terminationReason = sub._step()
                if isinstance(terminationReason, EndSimulationAction):
                    yield terminationReason
                elif terminationReason is None:
                    newSubs.append(sub)
            self._subScenarios = newSubs
            if not newSubs:
                return
            yield None

    def _evaluateRecordedExprs(self, ty):
        if ty is RequirementType.record:
            place = '_recordedExprs'
        elif ty is RequirementType.recordInitial:
            place = '_recordedInitialExprs'
        elif ty is RequirementType.recordFinal:
            place = '_recordedFinalExprs'
        else:
            assert False, 'invalid record type requested'
        return self._evaluateRecordedExprsAt(place)

    def _evaluateRecordedExprsAt(self, place):
        values = {}
        for rec in getattr(self, place):
            values[rec.name] = rec.value()
        for sub in self._subScenarios:
            subvals = sub._evaluateRecordedExprsAt(place)
            values.update(subvals)
        return values

    def _checkAlwaysRequirements(self):
        for req in self._alwaysRequirements:
            if not req.isTrue():
                # always requirements should never be violated at time 0, since
                # they are enforced during scene sampling
                assert veneer.currentSimulation.currentTime > 0
                raise RejectSimulationException(str(req))
        for sub in self._subScenarios:
            sub._checkAlwaysRequirements()

    def _runMonitors(self):
        terminationReason = None
        for monitor in self._monitors:
            action = monitor.step()
            if isinstance(action, EndSimulationAction):
                terminationReason = action
                # do not exit early, since subsequent monitors could reject the simulation
        for sub in self._subScenarios:
            subreason = sub._runMonitors()
            if subreason is not None:
                terminationReason = subreason
        return terminationReason

    def _checkSimulationTerminationConditions(self):
        for req in self._terminateSimulationConditions:
            if req.isTrue():
                return req
        return None

    @property
    def _allAgents(self):
        agents = list(self._agents)
        for sub in self._subScenarios:
            agents.extend(sub._allAgents)
        return agents

    def _inherit(self, other):
        self._objects.extend(other._objects)
        self._agents.extend(other._agents)
        self._globalParameters.update(other._globalParameters)
        self._externalParameters.extend(other._externalParameters)
        self._requirements.extend(other._requirements)
        self._behaviors.extend(other._behaviors)

    def _registerObject(self, obj):
        self._objects.append(obj)
        if getattr(obj, 'behavior', None) is not None:
            self._agents.append(obj)

    def _addRequirement(self, ty, reqID, req, line, name, prob):
        """Save a requirement defined at compile-time for later processing."""
        assert reqID not in self._pendingRequirements
        preq = PendingRequirement(ty, req, line, prob, name, self._ego)
        self._pendingRequirements[reqID] = preq

    def _addDynamicRequirement(self, ty, req, line, name):
        """Add a requirement defined during a dynamic simulation."""
        assert ty is not RequirementType.require
        dreq = DynamicRequirement(ty, req, line, name)
        self._registerCompiledRequirement(dreq)

    def _compileRequirements(self):
        namespace = self._dummyNamespace if self._dummyNamespace else self.__dict__
        requirementSyntax = self._requirementSyntax
        assert requirementSyntax is not None
        for reqID, requirement in self._pendingRequirements.items():
            syntax = requirementSyntax[reqID] if requirementSyntax else None
            compiledReq = requirement.compile(namespace, self, syntax)

            self._registerCompiledRequirement(compiledReq)
            self._requirementDeps.update(compiledReq.dependencies)

    def _registerCompiledRequirement(self, req):
        if req.ty is RequirementType.require:
            place = self._requirements
        elif req.ty is RequirementType.requireAlways:
            place = self._alwaysRequirements
        elif req.ty is RequirementType.terminateWhen:
            place = self._terminationConditions
        elif req.ty is RequirementType.terminateSimulationWhen:
            place = self._terminateSimulationConditions
        elif req.ty is RequirementType.record:
            place = self._recordedExprs
        elif req.ty is RequirementType.recordInitial:
            place = self._recordedInitialExprs
        elif req.ty is RequirementType.recordFinal:
            place = self._recordedFinalExprs
        else:
            raise RuntimeError(f'internal error: requirement {req} has unknown type!')
        place.append(req)

    def _setTimeLimit(self, timeLimit, inSeconds=True):
        self._timeLimit = timeLimit
        self._timeLimitIsInSeconds = inSeconds

    def _toScenario(self, namespace):
        assert self._prepared

        if self._ego is None and self._compose is None:
            msg = 'did not specify ego object'
            modScenarios = namespace['_scenarios']
            if self._dummyNamespace and len(modScenarios) == 1:
                if modScenarios[0]._requiresArguments():
                    msg += ('\n(Note: this Scenic file contains a modular scenario, but it\n'
                            'cannot be used as the top-level scenario since it requires\n'
                            'arguments; so the whole file is being used as a top-level\n'
                            'scenario and needs an ego object.)')
                else:
                    msg += ('\n(Note: this Scenic file contains a modular scenario, but also\n'
                            'other code; so the whole file is being used as a top-level\n'
                            'scenario and needs an ego object.)')
            raise InvalidScenarioError(msg)

        # Extract workspace, if one is specified
        if 'workspace' in namespace:
            workspace = namespace['workspace']
            if not isinstance(workspace, Workspace):
                raise InvalidScenarioError(f'workspace {workspace} is not a Workspace')
            if needsSampling(workspace):
                raise InvalidScenarioError('workspace must be a fixed region')
            if needsLazyEvaluation(workspace):
                raise InvalidScenarioError('workspace uses value undefined '
                                           'outside of object definition')
        else:
            workspace = None

        from scenic.core.scenarios import Scenario
        scenario = Scenario(workspace, self._simulatorFactory,
                            self._objects, self._ego,
                            self._globalParameters, self._externalParameters,
                            self._requirements, self._requirementDeps,
                            self._monitors, self._behaviorNamespaces,
                            self)   # TODO unify these!
        return scenario

    def __str__(self):
        if self._dummyNamespace:
            return 'top-level scenario'
        else:
            args = argsToString(itertools.chain(self._args, self._kwargs.items()))
            return self.__class__.__name__ + args

# Behaviors

class Behavior(Invocable, Samplable):
    def __init_subclass__(cls):
        if cls.__module__ is not __name__:
            veneer.currentScenario._behaviors.append(cls)

    def __init__(self, *args, **kwargs):
        self.args = tuple(toDistribution(arg) for arg in args)
        self.kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }
        if not inspect.isgeneratorfunction(self.makeGenerator):
            raise RuntimeParseError(f'{self} does not take any actions'
                                    ' (perhaps you forgot to use "take" or "do"?)')

        # Validate arguments to the behavior
        sig = inspect.signature(self.makeGenerator)
        try:
            sig.bind(None, *args, **kwargs)
        except TypeError as e:
            raise RuntimeParseError(str(e)) from e
        Samplable.__init__(self, itertools.chain(self.args, self.kwargs.values()))
        Invocable.__init__(self)

    def sampleGiven(self, value):
        args = (value[arg] for arg in self.args)
        kwargs = { name: value[val] for name, val in self.kwargs.items() }
        return type(self)(*args, **kwargs)

    def start(self, agent):
        self.agent = agent
        self._runningIterator = self.makeGenerator(agent, *self.args, **self.kwargs)

    def step(self):
        assert self._runningIterator
        with veneer.executeInBehavior(self):
            try:
                actions = self._runningIterator.send(None)
            except StopIteration:
                actions = ()    # behavior ended early
        return actions

    def stop(self):
        self.agent = None
        self._runningIterator = None

    @property
    def _isFinished(self):
        return self._runningIterator is None

    def _invokeInner(self, agent, subs):
        assert len(subs) == 1
        sub = subs[0]
        if not isinstance(sub, Behavior):
            raise RuntimeParseError(f'expected a behavior, got {sub}')
        sub.start(agent)
        with veneer.executeInBehavior(sub):
            try:
                yield from sub._runningIterator
            finally:
                sub.stop()

    def __str__(self):
        args = argsToString(itertools.chain(self.args, self.kwargs.items()))
        return self.__class__.__name__ + args

def makeTerminationAction(line):
    assert not veneer.isActive()
    return EndSimulationAction(line)

# Monitors

class Monitor(Behavior):
    def __init_subclass__(cls):
        super().__init_subclass__()
        veneer.currentScenario._monitors.append(cls())

    def start(self):
        return super().start(None)

monitorPrefix = '_Scenic_monitor_'
def functionForMonitor(name):
    return monitorPrefix + name
def isAMonitorName(name):
    return name.startswith(monitorPrefix)
def monitorName(name):
    return name[len(monitorPrefix):]

# Guards

class GuardViolation(Exception):
    """Abstract exception raised when a guard of a behavior is violated.

    This will never be raised directly; either of the subclasses `PreconditionViolation`
    or `InvariantViolation` will be used, as appropriate.
    """
    violationType = 'guard'

    def __init__(self, behavior, lineno):
        self.behaviorName = behavior.__class__.__name__
        self.lineno = lineno

    def __str__(self):
        return f'violated {self.violationType} of {self.behaviorName} on line {self.lineno}'

class PreconditionViolation(GuardViolation):
    """Raised when a precondition is violated when invoking a behavior."""
    violationType = 'precondition'

class InvariantViolation(GuardViolation):
    """Raised when an invariant is violated when invoking/resuming a behavior."""
    violationType = 'invariant'

# Try-interrupt blocks

def runTryInterrupt(behavior, agent, body, conditions, handlers):
    body = InterruptBlock(None, body)
    interrupts = [InterruptBlock(c, h) for c, h in zip(conditions, handlers)]
    while True:
        # find next block to run, if any
        block = body
        for interrupt in interrupts:
            if interrupt.isEnabled or interrupt.isRunning:
                block = interrupt
                break
        result, concluded = block.step(behavior, agent)
        if concluded:
            if result is BlockConclusion.FINISHED and block is not body:
                continue    # interrupt handler finished
            else:
                return result   # entire try-interrupt statement will terminate
        else:
            yield result
            behavior.checkInvariants()

@enum.unique
class BlockConclusion(enum.Enum):
    FINISHED = enum.auto()
    ABORT = enum.auto()
    RETURN = enum.auto()
    BREAK = enum.auto()
    CONTINUE = enum.auto()

    def __call__(self, value):
        self.return_value = value
        return self

class InterruptBlock:
    def __init__(self, condition, body):
        self.condition = condition
        self.body = body
        self.runningIterator = None

    @property
    def isEnabled(self):
        with veneer.executeInGuard():
            return bool(self.condition())

    @property
    def isRunning(self):
        return self.runningIterator is not None

    def step(self, behavior, agent):
        if not self.runningIterator:
            it = self.body(behavior, agent)
            if not isinstance(it, types.GeneratorType):
                return (it, True)
            self.runningIterator = it
        try:
            result = self.runningIterator.send(None)
            return (result, False)
        except StopIteration as e:
            self.runningIterator = None
            return (e.value, True)

import scenic.syntax.veneer as veneer
