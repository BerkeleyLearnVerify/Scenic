"""General code for invocables, i.e. behaviors, monitors, and modular scenarios."""

import enum
import types

from scenic.core.distributions import Options
from scenic.core.errors import InvalidScenarioError
from scenic.core.lazy_eval import DelayedArgument, needsLazyEvaluation

from .guards import GuardViolation
from .utils import RejectSimulationException


class Invocable:
    """Abstract class with common code for behaviors and modular scenarios.

    Both of these types of objects can be called like functions, can have guards, and can
    suspend their own execution to invoke sub-behaviors/scenarios.
    """

    def __init__(self, *args, **kwargs):
        import scenic.syntax.veneer as veneer

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
                import scenic.syntax.veneer as veneer

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
        import scenic.syntax.veneer as veneer

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
