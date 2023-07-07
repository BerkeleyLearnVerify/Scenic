"""Support for dynamic behaviors and modular scenarios."""

from collections import defaultdict
import dataclasses
import enum
import functools
import inspect
import itertools
import sys
import types
import warnings
import weakref

import rv_ltl

from scenic.core.distributions import Options, Samplable, needsSampling, toDistribution
from scenic.core.errors import InvalidScenarioError
from scenic.core.lazy_eval import DelayedArgument, needsLazyEvaluation
from scenic.core.requirements import (
    DynamicRequirement,
    PendingRequirement,
    RequirementType,
)
from scenic.core.simulators import (
    EndScenarioAction,
    EndSimulationAction,
    RejectSimulationException,
)
from scenic.core.type_support import CoercionFailure
from scenic.core.utils import alarm, argsToString
from scenic.core.workspaces import Workspace

# Utilities


class StuckBehaviorWarning(UserWarning):
    """Warning issued when a behavior/scenario may have gotten stuck.

    When a behavior or compose block of a modular scenario executes for a long
    time without yielding control, there is no way to tell whether it has
    entered an infinite loop with no take/wait statements, or is actually doing
    some long computation. But since forgetting a wait statement in a wait loop
    is an easy mistake, we raise this warning after a behavior/scenario has run
    for `stuckBehaviorWarningTimeout` seconds without yielding.
    """

    pass


#: Timeout in seconds after which a `StuckBehaviorWarning` will be raised.
stuckBehaviorWarningTimeout = 10

# Scenarios


class Invocable:
    """Abstract class with common code for behaviors and modular scenarios.

    Both of these types of objects can be called like functions, can have guards, and can
    suspend their own execution to invoke sub-behaviors/scenarios.
    """

    def __init__(self, *args, **kwargs):
        if veneer.evaluatingGuard:
            raise InvalidScenarioError(
                "tried to invoke behavior/scenario from inside guard or interrupt condition"
            )
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
                    if sub._isEnabledForAgent(agent):
                        enabled[sub] = weight
            else:
                for sub in opts:
                    if sub._isEnabledForAgent(agent):
                        enabled[sub] = 1
            if not enabled:
                raise RejectSimulationException('deadlock in "do choose/shuffle"')
            if len(enabled) == 1:
                choice = list(enabled)[0]
            else:
                choice = Options(enabled)
            return choice

        scheduler = None
        if schedule == "choose":
            if len(subs) == 1 and isinstance(subs[0], dict):
                subs = subs[0]
            subs = (pickEnabledInvocable(subs),)
        elif schedule == "shuffle":
            if len(subs) == 1 and isinstance(subs[0], dict):
                subs = subs[0]
            else:
                subs = {item: 1 for item in subs}

            def scheduler():
                while subs:
                    choice = pickEnabledInvocable(subs)
                    subs.pop(choice)
                    yield from self._invokeInner(agent, (choice,))

        else:
            assert schedule is None
        if not scheduler:

            def scheduler():
                yield from self._invokeInner(agent, subs)

        if modifier:
            if modifier.name == "for":  # do X for Y [seconds | steps]
                timeLimit = modifier.value
                if not isinstance(timeLimit, (float, int)):
                    raise TypeError('"do X for Y" with Y not a number')
                assert modifier.terminator in (None, "seconds", "steps")
                if modifier.terminator != "steps":
                    timeLimit /= veneer.currentSimulation.timestep
                startTime = veneer.currentSimulation.currentTime
                condition = (
                    lambda: veneer.currentSimulation.currentTime - startTime >= timeLimit
                )
            elif modifier.name == "until":  # do X until Y
                condition = modifier.value
            else:
                raise RuntimeError(
                    f"internal parsing error: impossible modifier {modifier}"
                )

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

    def _isEnabledForAgent(self, agent):
        assert not self._isRunning
        assert self._agent is None
        try:
            self._agent = agent  # in case `self` is used in a precondition
            self._checkAllPreconditions()
            return True
        except GuardViolation:
            return False
        finally:
            self._agent = None


class DynamicScenario(Invocable):
    """Internal class for scenarios which can execute during dynamic simulations.

    Provides additional information complementing `Scenario`, which originally only
    supported static scenarios. The two classes should probably eventually be merged.
    """

    def __init_subclass__(cls, *args, **kwargs):
        veneer.registerDynamicScenarioClass(cls)

        target = cls._setup or cls._compose or (lambda self, agent: 0)
        target = functools.partial(target, 0, 0)  # account for Scenic-inserted args
        cls.__signature__ = inspect.signature(target)

    _requirementSyntax = None  # overridden by subclasses
    _simulatorFactory = None
    _globalParameters = None
    _locals = ()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ego = None
        self._workspace = None
        self._instances = []  # ordered for reproducibility
        # _objects should contain a reference to the most complete version of
        # the objects in this scene (sampled > unsampled)
        self._objects = []  # ordered for reproducibility
        self._sampledObjects = self._objects
        self._externalParameters = []
        self._pendingRequirements = defaultdict(list)
        self._requirements = []
        # things needing to be sampled to evaluate the requirements
        self._requirementDeps = set()

        self._agents = []
        self._monitors = []
        self._behaviors = []
        self._monitorRequirements = []
        self._temporalRequirements = []
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

        self._timeLimitInSteps = None  # computed at simulation time
        self._elapsedTime = 0
        self._eventuallySatisfied = None
        self._overrides = {}

        self._requirementMonitors = None

    @classmethod
    def _dummy(cls, namespace):
        scenario = cls()
        scenario._setup = None
        scenario._compose = None
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
            sig.bind(None, None)  # first two arguments are added internally by Scenic
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
        self._workspace = scene.workspace
        self._objects = list(scene.objects)
        self._agents = [obj for obj in scene.objects if obj.behavior is not None]
        self._monitors = list(scene.monitors)
        self._temporalRequirements = scene.temporalRequirements
        self._terminationConditions = scene.terminationConditions
        self._terminateSimulationConditions = scene.terminateSimulationConditions
        self._recordedExprs = scene.recordedExprs
        self._recordedInitialExprs = scene.recordedInitialExprs
        self._recordedFinalExprs = scene.recordedFinalExprs

    def _prepare(self, delayPreconditionCheck=False):
        """Prepare the scenario for execution, executing its setup block."""
        assert not self._prepared
        self._prepared = True

        self._finalizeArguments()  # TODO generalize _prepare for Invocable?

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

        # create monitors for each requirement used for this simulation
        self._requirementMonitors = [r.toMonitor() for r in self._temporalRequirements]

        veneer.startScenario(self)
        with veneer.executeInScenario(self):
            # Start compose block
            if self._compose is not None:
                if not inspect.isgeneratorfunction(self._compose):
                    from scenic.syntax.translator import composeBlock

                    raise InvalidScenarioError(
                        f'"{composeBlock}" does not invoke any scenarios'
                    )
                self._runningIterator = self._compose(None, *self._args, **self._kwargs)

            # Initialize behavior coroutines of agents
            for agent in self._agents:
                behavior = agent.behavior
                assert isinstance(behavior, Behavior), behavior
                behavior._assignTo(agent)
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

        # Check temporal requirements
        for m in self._requirementMonitors:
            result = m.value()
            if result == rv_ltl.B4.FALSE:
                raise RejectSimulationException(str(m))

        # Check if we have reached the time limit, if any
        if (
            self._timeLimitInSteps is not None
            and self._elapsedTime >= self._timeLimitInSteps
        ):
            return self._stop("reached time limit")
        self._elapsedTime += 1

        # Execute compose block, if any
        composeDone = False
        if self._runningIterator is None:
            composeDone = True  # compose block ended in an earlier step
        else:

            def alarmHandler(signum, frame):
                if sys.gettrace():
                    return  # skip the warning if we're in the debugger
                warnings.warn(
                    f"the compose block of scenario {self} is taking a long time; "
                    'maybe you have an infinite loop with no "wait" statement?',
                    StuckBehaviorWarning,
                )

            with veneer.executeInScenario(self), alarm(
                stuckBehaviorWarningTimeout, alarmHandler
            ):
                try:
                    result = self._runningIterator.send(None)
                    if isinstance(result, (EndSimulationAction, EndScenarioAction)):
                        return self._stop(result)
                except StopIteration:
                    self._runningIterator = None
                    composeDone = True

        # If there is a compose block and it has finished, we're done
        if self._compose is not None and composeDone:
            return self._stop("finished compose block")

        # Optionally end when all our agents' behaviors have ended
        if self._endWithBehaviors:
            if all(agent.behavior._isFinished for agent in self._agents):
                return self._stop("all behaviors finished")

        # Check if any termination conditions apply
        for req in self._terminationConditions:
            if req.evaluate():
                return self._stop(req)

        # Scenario will not terminate yet
        return None

    def _stop(self, reason, quiet=False):
        """Stop the scenario's execution, for the given reason."""
        assert self._isRunning

        # Stop monitors and subscenarios.
        for monitor in self._monitors:
            if monitor._isRunning:
                monitor._stop()
        self._monitors = []
        for sub in self._subScenarios:
            if sub._isRunning:
                sub._stop("parent scenario ending", quiet=quiet)
        self._runningIterator = None

        # Revert overrides.
        for obj, oldVals in self._overrides.items():
            obj._revert(oldVals)

        # Inform the veneer we have stopped, and mark ourselves finished.
        veneer.endScenario(self, reason, quiet=quiet)
        super()._stop(reason)

        # Reject if a temporal requirement was not satisfied.
        if not quiet:
            for req in self._requirementMonitors:
                if req.lastValue.is_falsy:
                    raise RejectSimulationException(str(req))
        self._requirementMonitors = None

        return reason

    def _invokeInner(self, agent, subs):
        for sub in subs:
            if not isinstance(sub, DynamicScenario):
                raise TypeError(f"expected a scenario, got {sub}")
            sub._prepare()
            sub._start()
        self._subScenarios = list(subs)
        while True:
            newSubs = []
            for sub in self._subScenarios:
                terminationReason = sub._step()
                if isinstance(terminationReason, EndSimulationAction):
                    yield terminationReason
                    assert False, self  # should never get here since simulation ends
                elif terminationReason is None:
                    newSubs.append(sub)
            self._subScenarios = newSubs
            if not newSubs:
                return
            yield None
            # Check if any sub-scenarios stopped during action execution
            self._subScenarios = [sub for sub in self._subScenarios if sub._isRunning]

    def _evaluateRecordedExprs(self, ty):
        if ty is RequirementType.record:
            place = "_recordedExprs"
        elif ty is RequirementType.recordInitial:
            place = "_recordedInitialExprs"
        elif ty is RequirementType.recordFinal:
            place = "_recordedFinalExprs"
        else:
            assert False, "invalid record type requested"
        return self._evaluateRecordedExprsAt(place)

    def _evaluateRecordedExprsAt(self, place):
        values = {}
        for rec in getattr(self, place):
            values[rec.name] = rec.evaluate()
        for sub in self._subScenarios:
            subvals = sub._evaluateRecordedExprsAt(place)
            values.update(subvals)
        return values

    def _runMonitors(self):
        terminationReason = None
        endScenario = None
        for monitor in self._monitors:
            action = monitor._step()
            # do not exit early, since subsequent monitors could reject the simulation
            if isinstance(action, EndSimulationAction):
                terminationReason = action
            elif isinstance(action, EndScenarioAction):
                assert action.scenario is None
                endScenario = action
        for sub in self._subScenarios:
            subreason = sub._runMonitors()
            if subreason is not None:
                terminationReason = subreason
        if endScenario:
            self._stop(endScenario)
        return terminationReason or endScenario

    def _checkSimulationTerminationConditions(self):
        for req in self._terminateSimulationConditions:
            if req.isTrue().is_truthy:
                return req
        return None

    @property
    def _allAgents(self):
        agents = list(self._agents)
        for sub in self._subScenarios:
            agents.extend(sub._allAgents)
        return agents

    def _inherit(self, other):
        if not self._workspace:
            self._workspace = other._workspace
        self._instances.extend(other._instances)
        self._objects.extend(other._objects)
        self._agents.extend(other._agents)
        self._globalParameters.update(other._globalParameters)
        self._externalParameters.extend(other._externalParameters)
        self._requirements.extend(other._requirements)
        self._behaviors.extend(other._behaviors)

    def _registerInstance(self, inst):
        self._instances.append(inst)

    def _registerObject(self, obj):
        self._registerInstance(obj)
        self._objects.append(obj)
        if getattr(obj, "behavior", None) is not None:
            self._agents.append(obj)

        obj._parentScenario = weakref.ref(self)

    def _addRequirement(self, ty, reqID, req, line, name, prob):
        """Save a requirement defined at compile-time for later processing."""
        assert reqID not in self._pendingRequirements
        preq = PendingRequirement(ty, req, line, prob, name, self._ego)
        self._pendingRequirements[reqID] = preq

    def _addDynamicRequirement(self, ty, req, line, name):
        """Add a requirement defined during a dynamic simulation."""
        dreq = DynamicRequirement(ty, req, line, name)
        self._temporalRequirements.append(dreq)

    def _addMonitor(self, monitor):
        """Add a monitor during a dynamic simulation."""
        assert isinstance(monitor, Monitor)
        self._monitors.append(monitor)
        if self._isRunning:
            monitor._start()

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
        elif req.ty is RequirementType.monitor:
            place = self._monitorRequirements
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
            raise RuntimeError(f"internal error: requirement {req} has unknown type!")
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

        if not self._workspace:
            self._workspace = Workspace()  # default empty workspace
        astHash = namespace["_astHash"]
        name = None if self._dummyNamespace else self.__class__.__name__
        options = dataclasses.replace(namespace["_compileOptions"], scenario=name)

        from scenic.core.scenarios import Scenario

        scenario = Scenario(
            self._workspace,
            self._simulatorFactory,
            self._instances,
            self._objects,
            self._ego,
            self._globalParameters,
            self._externalParameters,
            self._requirements,
            self._requirementDeps,
            self._monitorRequirements,
            self._behaviorNamespaces,
            self,
            astHash,
            options,
        )  # TODO unify these!
        return scenario

    def __getattr__(self, name):
        if name in self._locals:
            return DelayedArgument(
                (), lambda context: getattr(self, name), _internal=True
            )
        return object.__getattribute__(self, name)

    def __str__(self):
        if self._dummyNamespace:
            return "top-level scenario"
        else:
            args = argsToString(self._args, self._kwargs)
            return f"{self.__class__.__name__}({args})"


# Behaviors


class Behavior(Invocable, Samplable):
    """Dynamic behaviors of agents.

    Behavior statements are translated into definitions of subclasses of this class.
    """

    def __init_subclass__(cls):
        if "__signature__" in cls.__dict__:
            # We're unpickling a behavior; skip this step.
            return

        if cls.__module__ is not __name__:
            if veneer.currentScenario:
                veneer.currentScenario._behaviors.append(cls)

            target = cls.makeGenerator
            target = functools.partial(target, 0, 0)  # account for Scenic-inserted args
            cls.__signature__ = inspect.signature(target)

    def __init__(self, *args, **kwargs):
        args = tuple(toDistribution(arg) for arg in args)
        kwargs = {name: toDistribution(arg) for name, arg in kwargs.items()}

        # Validate arguments to the behavior
        sig = inspect.signature(self.makeGenerator)
        sig.bind(None, *args, **kwargs)  # raises TypeError on incompatible arguments
        Samplable.__init__(self, itertools.chain(args, kwargs.values()))
        Invocable.__init__(self, *args, **kwargs)

        if not inspect.isgeneratorfunction(self.makeGenerator):
            raise InvalidScenarioError(
                f"{self} does not take any actions"
                ' (perhaps you forgot to use "take" or "do"?)'
            )

    @classmethod
    def _canCoerceType(cls, ty):
        return issubclass(ty, cls) or ty in (type, type(None))

    @classmethod
    def _coerce(cls, thing):
        if thing is None or isinstance(thing, cls):
            return thing
        elif issubclass(thing, cls):
            return thing()
        else:
            raise CoercionFailure(f"expected type of behavior, got {thing}")

    def sampleGiven(self, value):
        args = (value[arg] for arg in self._args)
        kwargs = {name: value[val] for name, val in self._kwargs.items()}
        return type(self)(*args, **kwargs)

    def _assignTo(self, agent):
        if self._agent and agent is self._agent._dynamicProxy:
            # Assigned again (e.g. by override) to same agent; do nothing.
            return
        if self._isRunning:
            raise InvalidScenarioError(
                f"tried to reuse behavior object {self} already assigned to {self._agent}"
            )
        self._start(agent)

    def _start(self, agent):
        super()._start()
        self._agent = agent
        self._runningIterator = self.makeGenerator(agent, *self._args, **self._kwargs)
        self._checkAllPreconditions()

    def _step(self):
        super()._step()
        assert self._runningIterator

        def alarmHandler(signum, frame):
            if sys.gettrace():
                return  # skip the warning if we're in the debugger
            warnings.warn(
                f"the behavior {self} is taking a long time to take an action; "
                "maybe you have an infinite loop with no take/wait statements?",
                StuckBehaviorWarning,
            )

        with veneer.executeInBehavior(self), alarm(
            stuckBehaviorWarningTimeout, alarmHandler
        ):
            try:
                actions = self._runningIterator.send(None)
            except StopIteration:
                actions = ()  # behavior ended early
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
            raise TypeError(f"expected a behavior, got {sub}")
        sub._start(agent)
        with veneer.executeInBehavior(sub):
            try:
                yield from sub._runningIterator
            finally:
                if sub._isRunning:
                    sub._stop()

    def __repr__(self):
        items = itertools.chain(
            (repr(arg) for arg in self._args),
            (f"{key}={repr(val)}" for key, val in self._kwargs.items()),
        )
        allArgs = ", ".join(items)
        return f"{self.__class__.__name__}({allArgs})"


def _makeTerminationAction(agent, line):
    assert not veneer.isActive()
    if agent:
        scenario = agent._parentScenario()
        assert scenario is not None
    else:
        scenario = None
    return EndScenarioAction(scenario, line)


def _makeSimulationTerminationAction(line):
    assert not veneer.isActive()
    return EndSimulationAction(line)


# Monitors


class Monitor(Behavior):
    """Monitors for dynamic simulations.

    Monitor statements are translated into definitions of subclasses of this class.
    """

    def _start(self):
        return super()._start(None)


# Guards


class GuardViolation(Exception):
    """Abstract exception raised when a guard of a behavior is violated.

    This will never be raised directly; either of the subclasses `PreconditionViolation`
    or `InvariantViolation` will be used, as appropriate.
    """

    violationType = "guard"

    def __init__(self, behavior, lineno):
        self.behaviorName = behavior.__class__.__name__
        self.lineno = lineno

    def __str__(self):
        return (
            f"violated {self.violationType} of {self.behaviorName} on line {self.lineno}"
        )


class PreconditionViolation(GuardViolation):
    """Exception raised when a precondition is violated

    Raised when a precondition is violated when invoking a behavior
    or when a precondition encounters a `RejectionException`, so that
    rejections count as precondition violations.
    """

    violationType = "precondition"


class InvariantViolation(GuardViolation):
    """Exception raised when an invariant is violated

    Raised when an invariant is violated when invoking/resuming a behavior
    or when an invariant encounters a `RejectionException`, so that
    rejections count as invariant violations.
    """

    violationType = "invariant"


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
                continue  # interrupt handler finished
            else:
                return result  # entire try-interrupt statement will terminate
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
