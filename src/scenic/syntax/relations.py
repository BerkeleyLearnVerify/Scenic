
"""Extracting relations (for later pruning) from the syntax of requirements."""

import math
from collections import defaultdict
from ast import Compare, BinOp, Eq, NotEq, Lt, LtE, Gt, GtE, Call, Add, Sub, Expression, Name

from scenic.core.distributions import needsSampling
from scenic.core.object_types import Point, Object
from scenic.core.errors import InvalidScenarioError, InconsistentScenarioError

def inferRelationsFrom(reqNode, namespace, ego, line):
    """Infer relations between objects implied by a requirement."""
    matcher = RequirementMatcher(namespace)

    inferRelativeHeadingRelations(matcher, reqNode, ego, line)
    inferDistanceRelations(matcher, reqNode, ego, line)

def inferRelativeHeadingRelations(matcher, reqNode, ego, line):
    """Infer bounds on relative headings from a requirement."""
    rhMatcher = lambda node: matcher.matchUnaryFunction('RelativeHeading', node)
    allBounds = matcher.matchBounds(reqNode, rhMatcher)
    for target, bounds in allBounds:
        if not isinstance(target, Object):
            continue
        assert target is not ego
        if ego is None:
            raise InvalidScenarioError('relative heading w.r.t. unassigned ego on line {line}')
        lower, upper = bounds
        if lower < -math.pi:
            lower = -math.pi
        if upper > math.pi:
            upper = math.pi
        if lower == -math.pi and upper == math.pi:
            continue    # skip trivial bounds
        rel = RelativeHeadingRelation(target, lower, upper)
        ego._relations.append(rel)
        conv = RelativeHeadingRelation(ego, -upper, -lower)
        target._relations.append(conv)

def inferDistanceRelations(matcher, reqNode, ego, line):
    """Infer bounds on distances from a requirement."""
    distMatcher = lambda node: matcher.matchUnaryFunction('DistanceFrom', node)
    allBounds = matcher.matchBounds(reqNode, distMatcher)
    for target, bounds in allBounds:
        if not isinstance(target, Object):
            continue
        assert target is not ego
        if ego is None:
            raise InvalidScenarioError('distance w.r.t. unassigned ego on line {line}')
        lower, upper = bounds
        if lower < 0:
            lower = 0
            if upper == float('inf'):
                continue    # skip trivial bounds
        rel = DistanceRelation(target, lower, upper)
        ego._relations.append(rel)
        conv = DistanceRelation(ego, lower, upper)
        target._relations.append(conv)

class BoundRelation:
    """Abstract relation bounding something about another object."""
    def __init__(self, target, lower, upper):
        self.target = target
        self.lower, self.upper = lower, upper

class RelativeHeadingRelation(BoundRelation):
    """Relation bounding another object's relative heading with respect to this one."""
    pass

class DistanceRelation(BoundRelation):
    """Relation bounding another object's distance from this one."""
    pass

class RequirementMatcher:
    def __init__(self, namespace):
        self.namespace = namespace

    def inconsistencyError(self, node, message):
        raise InconsistentScenarioError(node.lineno, message)

    def matchUnaryFunction(self, name, node):
        """Match a call to a specified unary function, returning the value of its argument."""
        if not (isinstance(node, Call) and isinstance(node.func, Name)
                and node.func.id == name):
            return None
        if len(node.args) != 1:
            return None
        if len(node.keywords) != 0:
            return None
        return self.matchValue(node.args[0])

    def matchBounds(self, node, matchAtom):
        """Match upper/lower bounds on something matched by the given function.

        Returns a list of all bounds found, pairing the bounded quantity with a
        pair (low, high) of lower/upper bounds.
        """
        if not isinstance(node, Compare):
            return {}
        bounds = defaultdict(lambda: (float('-inf'), float('inf')))
        targets = {}
        first = node.left
        for second, op in zip(node.comparators, node.ops):
            lower, upper, target = self.matchBoundsInner(first, second, op, matchAtom)
            first = second
            if target is None:
                continue
            targetID = id(target)    # use id to support unhashable types
            targets[targetID] = target
            bestLower, bestUpper = bounds[targetID]
            if lower is not None and lower > bestLower:
                bestLower = lower
            if upper is not None and upper < bestUpper:
                bestUpper = upper
            bounds[targetID] = (bestLower, bestUpper)
        return [(target, bounds[id_]) for id_, target in targets.items()]

    def matchBoundsInner(self, left, right, op, matchAtom):
        """Extract bounds from a single comparison operator."""
        # Reduce > and >= to < and <=
        if isinstance(op, Gt):
            return self.matchBoundsInner(right, left, Lt(), matchAtom)
        elif isinstance(op, GtE):
            return self.matchBoundsInner(right, left, LtE(), matchAtom)
        # Try matching a constant lower bound on the atom or its absolute value
        lconst = self.matchConstant(left)
        if isinstance(lconst, (int, float)):
            target = matchAtom(right)
            if target is not None:     # CONST op QUANTITY
                return (lconst, lconst, target) if isinstance(op, Eq) else (lconst, None, target)
            else:
                bounds = self.matchAbsBounds(right, lconst, op, False, matchAtom)
                if bounds is not None:      # CONST op abs(QUANTITY [+/- CONST])
                    return bounds
        # Try matching a constant upper bound on the atom or its absolute value
        rconst = self.matchConstant(right)
        if isinstance(rconst, (int, float)):
            target = matchAtom(left)
            if target is not None:      # QUANTITY op CONST
                return (rconst, rconst, target) if isinstance(op, Eq) else (None, rconst, target)
            else:
                bounds = self.matchAbsBounds(left, rconst, op, True, matchAtom)
                if bounds is not None:      # abs(QUANTITY [+/- CONST]) op CONST
                    return bounds
        return None, None, None

    def matchAbsBounds(self, node, const, op, isUpperBound, matchAtom):
        """Extract bounds on an atom from a comparison involving its absolute value."""
        if not (isinstance(node, Call) and isinstance(node.func, Name)
                and node.func.id == 'abs'):
            return None     # not an invocation of abs
        if not isUpperBound and not isinstance(op, Eq):
            return None     # lower bounds on abs value don't bound underlying quantity
        if const < 0:
            self.inconsistencyError(node, f'absolute value cannot be negative')
        assert len(node.args) == 1
        arg = node.args[0]
        target = matchAtom(arg)
        if target is not None:   # abs(QUANTITY) </= CONST
            return (-const, const, target)
        elif isinstance(arg, BinOp) and isinstance(arg.op, (Add, Sub)):   # abs(X +/- Y) </= CONST
            match = None
            slconst = self.matchConstant(arg.left)
            target = matchAtom(arg.right)
            if (isinstance(slconst, (int, float))
                and target is not None):   # abs(CONST +/- QUANTITY) </= CONST
                match = slconst
            else:
                srconst = self.matchConstant(arg.right)
                target = matchAtom(arg.left)
                if (isinstance(srconst, (int, float))
                    and target is not None):    # abs(QUANTITY +/- CONST) </= CONST
                    match = srconst
            if match is not None:
                if isinstance(arg.op, Add):    # abs(QUANTITY + CONST) </= CONST
                    return (-const - match, const - match, target)
                else:   # abs(QUANTITY - CONST) </= CONST
                    return (-const + match, const + match, target)
        return None

    def matchConstant(self, node):
        """Match constant values, i.e. values known prior to sampling."""
        value = self.matchValue(node)
        return None if needsSampling(value) else value

    def matchValue(self, node):
        """Match any expression which can be evaluated, returning its value.

        This method could have undesirable side-effects, but conditions in
        requirements should not have side-effects to begin with.
        """
        try:
            code = compile(Expression(node), '<internal>', 'eval')
            value = eval(code, dict(self.namespace))
        except Exception:
            return None
        return value
