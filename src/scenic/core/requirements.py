"""Support for hard and soft requirements."""

import enum
import inspect
from functools import reduce

import rv_ltl

from scenic.core.distributions import Samplable, needsSampling
from scenic.core.errors import InvalidScenarioError
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.propositions import Atomic, PropositionNode

import scenic.syntax.relations as relations

@enum.unique
class RequirementType(enum.Enum):
    # requirements which must hold during initial sampling
    require = 'require'

    # requirements used only during simulation
    terminateWhen = 'terminate when'
    terminateSimulationWhen = 'terminate simulation when'

    # recorded values, which aren't requirements but are handled similarly
    record = 'record'
    recordInitial = 'record initial'
    recordFinal = 'record final'

    @property
    def constrainsSampling(self):
        return self in (self.require,)

class PendingRequirement:
    def __init__(self, ty, condition, line, prob, name, ego):
        self.ty = ty
        self.condition = condition
        self.line = line
        self.prob = prob
        self.name = name

        # the translator wrapped the requirement in a lambda to prevent evaluation,
        # so we need to save the current values of all referenced names; we save
        # the ego object too since it can be referred to implicitly

        # condition is an instance of Proposition. Flatten to get a list of atomic propositions.
        nodes = condition.flatten()
        binding_list = []
        for node in nodes:
            if isinstance(node, Atomic):
                binding_list.append(getAllGlobals(node.closure))
        bindings = reduce(lambda d1, d2: {**d1, **d2}, binding_list, {})

        self.bindings = bindings

        self.egoObject = ego

    def compile(self, namespace, scenario, propositionSyntax=[]):
        """Create a closure testing the requirement in the correct runtime state.

        While we're at it, determine whether the requirement implies any relations
        we can use for pruning, and gather all of its dependencies.
        """
        bindings, ego, line = self.bindings, self.egoObject, self.line

        # Check whether requirement implies any relations used for pruning
        propositionIdForPruning = self.condition.check_constrains_sampling()
        if propositionIdForPruning is not None and propositionIdForPruning < len(propositionSyntax):
            syntaxForPruning = propositionSyntax[propositionIdForPruning]
            relations.inferRelationsFrom(syntaxForPruning, bindings, ego, line)

        # Gather dependencies of the requirement
        deps = set()
        for value in bindings.values():
            if needsSampling(value):
                deps.add(value)
            if needsLazyEvaluation(value):
                raise InvalidScenarioError(f'{self.ty} on line {line} uses value {value}'
                                           ' undefined outside of object definition')
        if ego is not None:
            assert isinstance(ego, Samplable)
            deps.add(ego)

        # Construct closure
        def closure(values, monitor = None):
            global evaluatingRequirement, currentScenario
            # rebind any names referring to sampled objects
            for name, value in bindings.items():
                if value in values:
                    namespace[name] = values[value]
            # rebind ego object, which can be referred to implicitly
            boundEgo = None if ego is None else values[ego]
            # evaluate requirement condition, reporting errors on the correct line
            import scenic.syntax.veneer as veneer
            with veneer.executeInRequirement(scenario, boundEgo):
                if monitor is None:
                    # if not temporal evaluation
                    result = self.condition.evaluate()
                else:
                    # if temporal evaluation
                    result = monitor.update()
                assert not needsSampling(result)
                if needsLazyEvaluation(result):
                    raise InvalidScenarioError(f'{self.ty} on line {line} uses value'
                                               ' undefined outside of object definition')
            return result

        return CompiledRequirement(self, closure, deps, self.condition)

def getAllGlobals(req, restrictTo=None):
    """Find all names the given lambda depends on, along with their current bindings."""
    namespace = req.__globals__
    if restrictTo is not None and restrictTo is not namespace:
        return {}
    externals = inspect.getclosurevars(req)
    assert not externals.nonlocals      # TODO handle these
    globs = dict(externals.builtins)
    for name, value in externals.globals.items():
        globs[name] = value
        if inspect.isfunction(value):
            subglobs = getAllGlobals(value, restrictTo=namespace)
            for name, value in subglobs.items():
                if name in globs:
                    assert value is globs[name]
                else:
                    globs[name] = value
    return globs

class CompiledRequirement:
    def __init__(self, pendingReq, closure, dependencies, proposition):
        self.ty = pendingReq.ty
        self.closure = closure
        self.line = pendingReq.line
        self.name = pendingReq.name
        self.prob = pendingReq.prob
        self.dependencies = dependencies
        self.proposition = proposition

    @property
    def constrainsSampling(self):
        return self.ty.constrainsSampling

    def satisfiedBy(self, sample):
        one_time_monitor = self.proposition.create_monitor()
        return self.closure(sample, one_time_monitor)

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'

class BoundRequirement:
    def __init__(self, compiledReq, sample, proposition):
        self.ty = compiledReq.ty
        self.closure = compiledReq.closure
        self.line = compiledReq.line
        self.name = compiledReq.name
        self.sample = sample
        self.compiledReq = compiledReq
        self.proposition = proposition

    def isTrue(self):
        return self.value()

    def value(self):
        one_time_monitor = self.proposition.create_monitor()
        return self.closure(self.sample, one_time_monitor)

    def evaluate(self):
        return self.closure(self.sample)

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'

    def toMonitor(self):
        return MonitorRequirement(self.compiledReq, self.sample, self.proposition)

class MonitorRequirement(BoundRequirement):
    """MonitorRequirement is a BoundRequirement with temporal proposition monitor
    """
    def __init__(self, compiledReq, sample, proposition):
        super().__init__(compiledReq, sample, proposition)
        self.monitor = self.proposition.create_monitor()
        self.lastValue = rv_ltl.B4.TRUE
    
    def value(self):
        self.lastValue = self.closure(self.sample, self.monitor)
        return self.lastValue

class DynamicRequirement:
    def __init__(self, ty, condition, line, name=None):
        self.ty = ty
        self.line = line
        self.name = name

        import scenic.syntax.veneer as veneer
        scenario = veneer.currentScenario
        def closure(monitor = None):
            with veneer.executeInScenario(scenario):
                if monitor is None:
                    result = self.condition.evaluate()
                else:
                    result = monitor.update()
                return result
        self.closure = closure
        self.condition = condition

    def isTrue(self):
        return self.value()

    def value(self):
        return self.closure()

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'

    def toMonitor(self):
        return DynamicMonitorRequirement(self.closure, self.condition, self.line, self.name)

class DynamicMonitorRequirement:
    def __init__(self, closure, condition, line, name):
        self.line = line
        self.closure = closure
        self.name = name
        self.condition = condition
        self.monitor = self.condition.create_monitor()
        self.lastValue = rv_ltl.B4.TRUE

    def isTrue(self):
        return self.value()

    def value(self):
        self.lastValue = self.closure(self.monitor)
        return self.lastValue

    def evaluate(self):
        return self.closure()

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'
