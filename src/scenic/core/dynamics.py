"""Support for dynamic behaviors and modular scenarios."""

from collections import defaultdict
import enum
import inspect
import itertools
import types

from scenic.core.distributions import Samplable, Options, toDistribution, needsSampling
from scenic.core.errors import RuntimeParseError, InvalidScenarioError
from scenic.core.lazy_eval import DelayedArgument, needsLazyEvaluation
from scenic.core.requirements import (RequirementType, PendingRequirement,
                                      DynamicRequirement)
from scenic.core.simulators import (RejectSimulationException, EndSimulationAction,
                                    EndScenarioAction)
from scenic.core.utils import argsToString
from scenic.core.workspaces import Workspace

# Scenarios

class Invocable:
    """Abstract class with common code for behaviors and modular scenarios.

    Both of these types of objects can be called like functions, can have guards, and can
    suspend their own execution to invoke sub-behaviors/scenarios.
    """
    def __init__(self, *args, **kwargs):
        if veneer.evaluatingGuard:
            raise RuntimeParseError(
                'tried to invoke behavior/scenario from inside guard or interrupt condition')
        self._args = args
        self._kwargs = kwargs
        self._agent = None
        self._runningIterator = None
        self._isRunning = False

    def _start(self):
        assert not self._isRunning
        self._isRunning = True
        self._finalizeArguments()

    def _step(self):
        assert self._isRunning

    def _stop(self, reason=None):
        assert self._isRunning
        self._isRunning = False

    def _finalizeArguments(self):
        # Evaluate any lazy arguments whose evaluation was deferred until just before
        # this invocable starts running.
        args = []
        for arg in self._args:
            if needsLazyEvaluation(arg):
                assert isinstance(arg, DelayedArgument)
                args.append(arg.evaluateInner(None))
            else:
                args.append(arg)
        self._args = tuple(args)
        for name, arg in self._kwargs.items():
            if needsLazyEvaluation(arg):
                assert isinstance(arg, DelayedArgument)
                self._kwargs[name] = arg.evaluateInner(None)

    def _invokeSubBehavior(self, agent, subs, modifier=None, schedule=None):
        def pickEnabledInvocable(opts):
            enabled = {}
            if isinstance(opts, dict):
                for sub, weight in opts.items():
                    if sub._isEnabled:
                        enabled[sub] = weight
            else:
                for sub in opts:
                    if sub._isEnabled:
                        enabled[sub] = 1
            if not enabled:
                raise RejectSimulationException('deadlock in "do choose/shuffle"')
            if len(enabled) == 1:
                choice = list(enabled)[0]
            else:
                choice = Options(enabled)
            return choice

        scheduler = None
        if schedule == 'choose':
            if len(subs) == 1 and isinstance(subs[0], dict):
                subs = subs[0]
            subs = (pickEnabledInvocable(subs),)
        elif schedule == 'shuffle':
            def scheduler():
                left = set(subs)
                while left:
                    choice = pickEnabledInvocable(left)
                    left.remove(choice)
                    yield from self._invokeInner(agent, (choice,))
        else:
            assert schedule is None
        if not scheduler:
            def scheduler():
                yield from self._invokeInner(agent, subs)

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
                yield from scheduler()
            def handler(behavior, agent):
                for sub in subs:
                    if sub._isRunning:
                        sub._stop(f'"{modifier.name}" condition met')
                return BlockConclusion.ABORT
            yield from runTryInterrupt(self, agent, body, [condition], [handler])
        else:
            yield from scheduler()

    def _invokeInner(self, agent, subs):
        """Run the given sub-behavior/scenario(s) in parallel.

        Implemented by subclasses.
        """
        raise NotImplementedError

    def _checkAllPreconditions(self):
        self.checkPreconditions(self._agent, *self._args, **self._kwargs)
        self.checkInvariants(self._agent, *self._args, **self._kwargs)

    @property
    def _isEnabled(self):
        try:
            self._checkAllPreconditions()
            return True
        except GuardViolation:
            return False

class DynamicScenario(Invocable):
    """Internal class for scenarios which can execute during dynamic simulations.

    Provides additional information complementing `Scenario`, which originally only
    supported static scenarios. The two classes should probably eventually be merged.
    """
    def __init_subclass__(cls, *args, **kwargs):
        veneer.registerDynamicScenarioClass(cls)

    _requirementSyntax = None   # overridden by subclasses
    _simulatorFactory = None
    _globalParameters = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
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
        self._eventuallyRequirements = []
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
        self._delayingPreconditionCheck = False
        self._dummyNamespace = None

        self._timeLimitInSteps = None   # computed at simulation time
        self._elapsedTime = 0
        self._eventuallySatisfied = None
        self._overrides = {}

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
        if self._ego is None:
            return DelayedArgument((), lambda context: self._ego, _internal=True)
        return self._ego

    @property
    def objects(self):
        return tuple(self._objects)

    def _bindTo(self, scene):
        """Bind this scenario to a sampled scene when starting a new simulation."""
        self._ego = scene.egoObject
        self._objects = list(scene.objects)
        self._agents = [obj for obj in scene.objects if obj.behavior is not None]
        self._alwaysRequirements = scene.alwaysRequirements
        self._eventuallyRequirements = scene.eventuallyRequirements
        self._terminationConditions = scene.terminationConditions
        self._terminateSimulationConditions = scene.terminateSimulationConditions
        self._recordedExprs = scene.recordedExprs
        self._recordedInitialExprs = scene.recordedInitialExprs
        self._recordedFinalExprs = scene.recordedFinalExprs

    def _prepare(self, delayPreconditionCheck=False):
        """Prepare the scenario for execution, executing its setup block."""
        assert not self._prepared
        self._prepared = True

        self._finalizeArguments()   # TODO generalize _prepare for Invocable?

        veneer.prepareScenario(self)
        with veneer.executeInScenario(self, inheritEgo=True):
            # Check preconditions and invariants
            if delayPreconditionCheck:
                self._delayingPreconditionCheck = True
            else:
                self._checkAllPreconditions()

            # Execute setup block
            if self._setup is not None:
                assert not any(needsLazyEvaluation(arg) for arg in self._args)
                assert not any(needsLazyEvaluation(arg) for arg in self._kwargs.values())
                self._setup(None, *self._args, **self._kwargs)
                veneer.finishScenarioSetup(self)

        # Extract requirements, scan for relations used for pruning, and create closures
        self._compileRequirements()

    @classmethod
    def _bindGlobals(cls, globs):
        cls._globalParameters = globs

    def _start(self):
        """Start the scenario, starting its compose block, behaviors, and monitors."""
        super()._start()
        assert self._prepared

        # Check preconditions if they could not be checked earlier
        if self._delayingPreconditionCheck:
            self._checkAllPreconditions()

        # Compute time limit now that we know the simulation timestep
        self._elapsedTime = 0
        self._timeLimitInSteps = self._timeLimit
        if self._timeLimitIsInSeconds:
            self._timeLimitInSteps /= veneer.currentSimulation.timestep

        # Keep track of which 'require eventually' conditions have been satisfied
        self._eventuallySatisfied = { req: False for req in self._eventuallyRequirements }

        veneer.startScenario(self)
        with veneer.executeInScenario(self):
            # Start compose block
            if self._compose is not None:
                if not inspect.isgeneratorfunction(self._compose):
                    from scenic.syntax.translator import composeBlock
                    raise RuntimeParseError(f'"{composeBlock}" does not invoke any scenarios')
                self._runningIterator = self._compose(None, *self._args, **self._kwargs)

            # Initialize behavior coroutines of agents
            for agent in self._agents:
                assert isinstance(agent.behavior, Behavior), agent.behavior
                agent.behavior._start(agent)
            # Initialize monitor coroutines
            for monitor in self._monitors:
                monitor._start()

    def _step(self):
        """Execute the (already-started) scenario for one time step.

        Returns:
            `None` if the scenario will continue executing; otherwise a string describing
            why it has terminated.
        """
        super()._step()

        # Check 'require always' and 'require eventually' conditions
        for req in self._alwaysRequirements:
            if not req.isTrue():
                # always requirements should never be violated at time 0, since
                # they are enforced during scene sampling
                assert veneer.currentSimulation.currentTime > 0
                raise RejectSimulationException(str(req))
        for req in self._eventuallyRequirements:
            if not self._eventuallySatisfied[req] and req.isTrue():
                self._eventuallySatisfied[req] = True

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

    def _stop(self, reason, quiet=False):
        """Stop the scenario's execution, for the given reason."""
        if not quiet:
            # Reject if we never satisfied a 'require eventually'
            for req in self._eventuallyRequirements:
                if not self._eventuallySatisfied[req] and not req.isTrue():
                    raise RejectSimulationException(str(req))

        super()._stop(reason)
        veneer.endScenario(self, reason, quiet=quiet)
        for obj, oldVals in self._overrides.items():
            obj._revert(oldVals)
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

    def _runMonitors(self):
        terminationReason = None
        for monitor in self._monitors:
            action = monitor._step()
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
        elif req.ty is RequirementType.requireEventually:
            place = self._eventuallyRequirements
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

    def _override(self, obj, specifiers):
        oldVals = obj._override(specifiers)
        if obj not in self._overrides:
            self._overrides[obj] = oldVals

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

    def __getattr__(self, name):
        if name in self._locals:
            return DelayedArgument((), lambda context: getattr(self, name), _internal=True)
        return object.__getattribute__(self, name)

    def __str__(self):
        if self._dummyNamespace:
            return 'top-level scenario'
        else:
            args = argsToString(itertools.chain(self._args, self._kwargs.items()))
            return self.__class__.__name__ + args

# Behaviors

class Behavior(Invocable, Samplable):
    """Dynamic behaviors of agents.

    Behavior statements are translated into definitions of subclasses of this class.
    """
    def __init_subclass__(cls):
        if cls.__module__ is not __name__:
            veneer.currentScenario._behaviors.append(cls)

    def __init__(self, *args, **kwargs):
        args = tuple(toDistribution(arg) for arg in args)
        kwargs = { name: toDistribution(arg) for name, arg in kwargs.items() }

        # Validate arguments to the behavior
        sig = inspect.signature(self.makeGenerator)
        try:
            sig.bind(None, *args, **kwargs)
        except TypeError as e:
            raise RuntimeParseError(str(e)) from e
        Samplable.__init__(self, itertools.chain(args, kwargs.values()))
        Invocable.__init__(self, *args, **kwargs)

        if not inspect.isgeneratorfunction(self.makeGenerator):
            raise RuntimeParseError(f'{self} does not take any actions'
                                    ' (perhaps you forgot to use "take" or "do"?)')

    def sampleGiven(self, value):
        args = (value[arg] for arg in self._args)
        kwargs = { name: value[val] for name, val in self._kwargs.items() }
        return type(self)(*args, **kwargs)

    def _start(self, agent):
        super()._start()
        self._agent = agent
        self._runningIterator = self.makeGenerator(agent, *self._args, **self._kwargs)
        self._checkAllPreconditions()

    def _step(self):
        super()._step()
        assert self._runningIterator
        with veneer.executeInBehavior(self):
            try:
                actions = self._runningIterator.send(None)
            except StopIteration:
                actions = ()    # behavior ended early
        return actions

    def _stop(self, reason=None):
        super()._stop(reason)
        self._agent = None
        self._runningIterator = None

    @property
    def _isFinished(self):
        return self._runningIterator is None

    def _invokeInner(self, agent, subs):
        assert len(subs) == 1
        sub = subs[0]
        if not isinstance(sub, Behavior):
            raise RuntimeParseError(f'expected a behavior, got {sub}')
        sub._start(agent)
        with veneer.executeInBehavior(sub):
            try:
                yield from sub._runningIterator
            finally:
                if sub._isRunning:
                    sub._stop()

    def __str__(self):
        args = argsToString(itertools.chain(self._args, self._kwargs.items()))
        return self.__class__.__name__ + args

def makeTerminationAction(line):
    assert not veneer.isActive()
    return EndSimulationAction(line)

# Monitors

class Monitor(Behavior):
    """Monitors for dynamic simulations.

    Monitor statements are translated into definitions of subclasses of this class.
    """
    def __init_subclass__(cls):
        super().__init_subclass__()
        veneer.currentScenario._monitors.append(cls())

    def _start(self):
        return super()._start(None)

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
            behavior.checkInvariants(None, *behavior._args, **behavior._kwargs)

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
            result = self.condition()
            if isinstance(result, DelayedArgument):
                # Condition cannot yet be evaluated because it depends on a scenario
                # local not yet initialized; we consider it to be false.
                return False
            return bool(result)

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
