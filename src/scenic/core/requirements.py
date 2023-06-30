"""Support for hard and soft requirements."""

from abc import ABC, abstractmethod
import enum
from functools import reduce
import inspect
import itertools

import rv_ltl
import trimesh

from scenic.core.distributions import Samplable, needsSampling
from scenic.core.errors import InvalidScenarioError
from scenic.core.lazy_eval import needsLazyEvaluation
from scenic.core.propositions import Atomic, PropositionNode
import scenic.syntax.relations as relations


@enum.unique
class RequirementType(enum.Enum):
    # requirements which must hold during initial sampling
    require = "require"

    # requirements used only during simulation
    monitor = "require monitor"
    terminateWhen = "terminate when"
    terminateSimulationWhen = "terminate simulation when"

    # recorded values, which aren't requirements but are handled similarly
    record = "record"
    recordInitial = "record initial"
    recordFinal = "record final"

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
        atoms = condition.atomics()
        bindings = {}
        atomGlobals = None
        for atom in atoms:
            bindings.update(getAllGlobals(atom.closure))
            globs = atom.closure.__globals__
            if atomGlobals is not None:
                assert globs is atomGlobals
            else:
                atomGlobals = globs
        self.bindings = bindings
        self.egoObject = ego

    def compile(self, namespace, scenario, syntax=None):
        """Create a closure testing the requirement in the correct runtime state.

        While we're at it, determine whether the requirement implies any relations
        we can use for pruning, and gather all of its dependencies.
        """
        bindings, ego, line = self.bindings, self.egoObject, self.line
        condition, ty = self.condition, self.ty

        # Check whether requirement implies any relations used for pruning
        canPrune = condition.check_constrains_sampling()
        if canPrune:
            relations.inferRelationsFrom(syntax, bindings, ego, line)

        # Gather dependencies of the requirement
        deps = set()
        for value in bindings.values():
            if needsSampling(value):
                deps.add(value)
            if needsLazyEvaluation(value):
                raise InvalidScenarioError(
                    f"{ty} on line {line} uses value {value}"
                    " undefined outside of object definition"
                )

        # If this requirement contains the CanSee specifier, we will need to sample all objects
        # to meet the dependencies.
        if "CanSee" in bindings:
            deps.update(scenario.objects)

        if ego is not None:
            assert isinstance(ego, Samplable)
            deps.add(ego)

        # Construct closure
        def closure(values, monitor=None):
            # rebind any names referring to sampled objects
            # note: need to extract namespace here rather than close over value
            # from above because of https://github.com/uqfoundation/dill/issues/532
            namespace = condition.atomics()[0].closure.__globals__
            for name, value in bindings.items():
                if value in values:
                    namespace[name] = values[value]
            # rebind ego object, which can be referred to implicitly
            boundEgo = None if ego is None else values[ego]
            # evaluate requirement condition, reporting errors on the correct line
            import scenic.syntax.veneer as veneer

            with veneer.executeInRequirement(scenario, boundEgo, values):
                if monitor is None:
                    # if not temporal evaluation
                    result = condition.evaluate()
                else:
                    # if temporal evaluation
                    result = monitor.update()
                assert not needsSampling(result)
                if needsLazyEvaluation(result):
                    raise InvalidScenarioError(
                        f"{ty} on line {line} uses value"
                        " undefined outside of object definition"
                    )
            return result

        return CompiledRequirement(self, closure, deps, condition)


def getAllGlobals(req, restrictTo=None):
    """Find all names the given lambda depends on, along with their current bindings."""
    namespace = req.__globals__
    if restrictTo is not None and restrictTo is not namespace:
        return {}
    externals = inspect.getclosurevars(req)
    assert not externals.nonlocals  # TODO handle these
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
    """MonitorRequirement is a BoundRequirement with temporal proposition monitor"""

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

        def closure(monitor=None):
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
        return DynamicMonitorRequirement(
            self.closure, self.condition, self.line, self.name
        )


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


## Builtin Requirements
class SamplingRequirement(ABC):
    """A requirement to be checked to validate a sample.

    Args:
        optional: Whether or not this requirement must be
            checked to validate the sample. Optional samples
            can be checked, and if ``False`` imply that the
            sample is invalid, but do not need to be checked
            if all non-optional requirements are satisfied.
    """

    def __init__(self, optional):
        self.optional = optional
        self.active = True

    @abstractmethod
    def falsifiedByInner(self, sample):
        """Returns False if the requirement is falsifed, True otherwise"""

    def falsifiedBy(self, sample):
        assert self.active
        return self.falsifiedByInner(sample)

    @property
    @abstractmethod
    def violationMsg(self):
        """Message to be printed if the requirement is violated"""
        pass


class IntersectionRequirement(SamplingRequirement):
    def __init__(self, objA, objB, optional=False):
        super().__init__(optional=optional)
        self.objA = objA
        self.objB = objB

    def falsifiedByInner(self, sample):
        objA = sample[self.objA]
        objB = sample[self.objB]
        if objA.allowCollisions or objB.allowCollisions:
            return False
        return objA.intersects(objB)

    @property
    def violationMsg(self):
        return f"Intersection violation: {self.objA} intersects {self.objB}"


class BlanketCollisionRequirement(SamplingRequirement):
    def __init__(self, objects, optional=True):
        super().__init__(optional=optional)
        self.objects = objects
        self._collidingObjects = None

    def falsifiedByInner(self, sample):
        objects = tuple(sample[obj] for obj in self.objects)
        cm = trimesh.collision.CollisionManager()
        for i, obj in enumerate(objects):
            if not obj.allowCollisions:
                cm.add_object(str(i), obj.occupiedSpace.mesh)
        collision, names = cm.in_collision_internal(return_names=True)

        if collision:
            self._collidingObjects = tuple(sorted(names))

        return collision

    @property
    def violationMsg(self):
        assert self._collidingObjects is not None
        objA_index, objB_index = map(int, self._collidingObjects[0])
        objA, objB = self.objects[objA_index], self.objects[objB_index]
        return f"Intersection violation: {objA} intersects {objB}"


class ContainmentRequirement(SamplingRequirement):
    def __init__(self, obj, container, optional=False):
        super().__init__(optional=optional)
        self.obj = obj
        self.container = container

    def falsifiedByInner(self, sample):
        obj = sample[self.obj]
        container = sample[self.container]
        return not container.containsObject(obj)

    @property
    def violationMsg(self):
        return f"Containment violation: {self.obj} is not contained in its container"


class VisibilityRequirement(SamplingRequirement):
    def __init__(self, source, target, objects, optional=False):
        super().__init__(optional=optional)
        self.source = source
        self.target = target
        self.potential_occluders = tuple(
            obj for obj in objects if obj is not self.source and obj is not self.target
        )

    def falsifiedByInner(self, sample):
        source = sample[self.source]
        target = sample[self.target]
        potential_occluders = tuple(sample[obj] for obj in self.potential_occluders)
        occluders = tuple(obj for obj in potential_occluders if obj.occluding)
        return not source.canSee(target, occludingObjects=occluders)

    @property
    def violationMsg(self):
        return f"Visibility violation: {self.target} is not visible from {self.source}"


class NonVisibilityRequirement(VisibilityRequirement):
    def falsifiedByInner(self, sample):
        return not super().falsifiedByInner(sample)

    @property
    def violationMsg(self):
        return f"Non-visibility violation: {self.target} is visible from {self.source}"


class CompiledRequirement(SamplingRequirement):
    def __init__(self, pendingReq, closure, dependencies, proposition):
        super().__init__(optional=False)
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

    def falsifiedByInner(self, sample):
        one_time_monitor = self.proposition.create_monitor()
        return self.closure(sample, one_time_monitor) == rv_ltl.B4.FALSE

    def __str__(self):
        if self.name:
            return self.name
        else:
            return f'"{self.ty.value}" on line {self.line}'

    @property
    def violationMsg(self):
        return f"User requirement violation: {self}"
