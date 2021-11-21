"""Support for hard and soft requirements."""

import enum
import inspect

from scenic.core.distributions import Samplable, needsSampling
from scenic.core.errors import InvalidScenarioError
from scenic.core.lazy_eval import needsLazyEvaluation
import scenic.syntax.relations as relations

@enum.unique
class RequirementType(enum.Enum):
    # requirements which must hold during initial sampling
    require = 'require'
    requireAlways = 'require always'
    requireEventually = 'require eventually'

    # requirements used only during simulation
    terminateWhen = 'terminate when'
    terminateSimulationWhen = 'terminate simulation when'

    # recorded values, which aren't requirements but are handled similarly
    record = 'record'
    recordInitial = 'record initial'
    recordFinal = 'record final'

    @property
    def constrainsSampling(self):
        return self in (self.require, self.requireAlways)

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
        self.bindings = getAllGlobals(condition)
        self.egoObject = ego

    def compile(self, namespace, scenario, syntax=None):
        """Create a closure testing the requirement in the correct runtime state.

        While we're at it, determine whether the requirement implies any relations
        we can use for pruning, and gather all of its dependencies.
        """
        bindings, ego, line = self.bindings, self.egoObject, self.line
        condition = self.condition

        # Check whether requirement implies any relations used for pruning
        if self.ty.constrainsSampling and syntax:
            relations.inferRelationsFrom(syntax, bindings, ego, line)

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
        def closure(values):
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
                result = condition()
                assert not needsSampling(result)
                if needsLazyEvaluation(result):
                    raise InvalidScenarioError(f'{self.ty} on line {line} uses value'
                                               ' undefined outside of object definition')
            return result

        return CompiledRequirement(self, closure, deps)

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
    def __init__(self, pendingReq, closure, dependencies):
        self.ty = pendingReq.ty
        self.closure = closure
        self.line = pendingReq.line
        self.name = pendingReq.name
        self.prob = pendingReq.prob
        self.dependencies = dependencies

    @property
    def constrainsSampling(self):
        return self.ty.constrainsSampling

    def satisfiedBy(self, sample):
        return self.closure(sample)

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'

class BoundRequirement:
    def __init__(self, compiledReq, sample):
        self.ty = compiledReq.ty
        self.closure = compiledReq.closure
        self.line = compiledReq.line
        self.name = compiledReq.name
        assert compiledReq.prob == 1
        self.sample = sample

    def isTrue(self):
        return self.value()

    def value(self):
        return self.closure(self.sample)

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'

class DynamicRequirement:
    def __init__(self, ty, condition, line, name=None):
        self.ty = ty
        self.line = line
        self.name = name

        import scenic.syntax.veneer as veneer
        scenario = veneer.currentScenario
        def closure():
            with veneer.executeInScenario(scenario):
                return condition()
        self.closure = closure

    def isTrue(self):
        return self.value()

    def value(self):
        return self.closure()

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'
