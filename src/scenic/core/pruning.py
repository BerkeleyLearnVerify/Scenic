"""Pruning parts of the sample space which violate requirements.

The top-level function here, `prune`, is called as the very last step of scenario
compilation (from `translator.constructScenarioFrom`).
"""

import builtins
import math
import time

import shapely.geometry
import shapely.geos

from scenic.core.distributions import (
    AttributeDistribution,
    FunctionDistribution,
    MethodDistribution,
    OperatorDistribution,
    Samplable,
    needsSampling,
    supportInterval,
    underlyingFunction,
)
from scenic.core.errors import InvalidScenarioError
from scenic.core.geometry import hypot, normalizeAngle, plotPolygon, polygonUnion
from scenic.core.object_types import Object, Point
import scenic.core.regions as regions
from scenic.core.regions import EmptyRegion, MeshSurfaceRegion, MeshVolumeRegion
from scenic.core.type_support import TypecheckedDistribution
from scenic.core.vectors import (
    PolygonalVectorField,
    VectorField,
    VectorMethodDistribution,
    VectorOperatorDistribution,
)
from scenic.core.workspaces import Workspace
from scenic.syntax.relations import DistanceRelation, RelativeHeadingRelation

### Utilities


def currentPropValue(obj, prop):
    """Get the current value of an object's property, taking into account prior pruning."""
    value = getattr(obj, prop)
    return value._conditioned if isinstance(value, Samplable) else value


def isMethodCall(thing, method):
    """Match calls to a given method, taking into account distribution decorators."""
    if not isinstance(thing, (MethodDistribution, VectorMethodDistribution)):
        return False
    return thing.method is underlyingFunction(method)


def isFunctionCall(thing, function):
    """Match calls to a given function, taking into account distribution decorators."""
    if not isinstance(thing, FunctionDistribution):
        return False
    return thing.function is underlyingFunction(function)


def matchInRegion(position):
    """Match uniform samples from a `Region`

    Returns the Region, if any, and a lower and upper bound
    on the distance the object will be placed along with any
    offset that should be added to the base.
    """
    # Case 1: Position is simply a point in a region
    if isinstance(position, regions.PointInRegionDistribution):
        reg = position.region
        if isinstance(reg, Workspace):
            reg = reg.region
        return reg, 0, 0, None

    # Case 2: Position is a point in a region with a vector offset.
    if isinstance(position, VectorOperatorDistribution) and position.operator in (
        "__add__",
        "__radd__",
    ):
        if isinstance(position.object, regions.PointInRegionDistribution):
            reg = position.object.region
            assert len(position.operands) == 1
            offset = position.operands[0]
            # TODO: Proper vector supportInterval calculations. Right now this gives us None
            # if value is not exact
            lower, upper = supportInterval(offset.norm())

            return reg, lower, upper, offset

    return None, 0, 0, None


def matchPolygonalField(heading, position):
    """Match orientation yaw defined by a `PolygonalVectorField` at the given position.

    Matches the yaw attribute of orientations exactly equal to a `PolygonalVectorField`,
    or offset by a bounded disturbance. Returns a triple consisting of the matched field
    if any, together with lower/upper bounds on the disturbance.
    """
    if isFunctionCall(heading, normalizeAngle):
        assert len(heading.arguments) == 1
        return matchPolygonalField(heading.arguments[0], position)

    if (
        isinstance(heading, TypecheckedDistribution)
        and heading._valueType is builtins.float
    ):
        return matchPolygonalField(heading._dist, position)

    if (
        isinstance(heading, AttributeDistribution) and heading.attribute == "yaw"
    ):  # TODO generalize to other 3D angles?
        orientation = heading.object
        if (
            isMethodCall(orientation, VectorField.__getitem__)
            and isinstance(orientation.object, PolygonalVectorField)
            and orientation.arguments == (position,)
        ):
            return orientation.object, 0, 0

    if isinstance(heading, OperatorDistribution) and heading.operator in (
        "__add__",
        "__radd__",
    ):
        field, lower, upper = matchPolygonalField(heading.object, position)
        if field is not None:
            assert len(heading.operands) == 1
            offset = heading.operands[0]
            ol, oh = supportInterval(offset)
            if ol is not None and oh is not None:
                return field, lower + ol, upper + oh

    return None, 0, 0


### Pruning procedures


def prune(scenario, verbosity=1):
    """Prune a `Scenario`, removing infeasible parts of the space.

    This function directly modifies the Distributions used in the Scenario,
    but leaves the conditional distribution under the scenario's requirements
    unchanged. See `Samplable.conditionTo`.

    Currently, the following pruning techniques are applied in order:

        * Pruning based on containment (`pruneContainment`)
        * Pruning based on relative heading bounds (`pruneRelativeHeading`)
    """
    if verbosity >= 1:
        print("  Pruning scenario...")
        startTime = time.time()

    pruneContainment(scenario, verbosity)
    pruneRelativeHeading(scenario, verbosity)

    if verbosity >= 1:
        totalTime = time.time() - startTime
        print(f"  Pruned scenario in {totalTime:.4g} seconds.")


## Pruning based on containment


def pruneContainment(scenario, verbosity):
    """Prune based on the requirement that individual Objects fit within their container.

    Specifically, if O is positioned uniformly (with a possible offset) in region B and
    has container C, then we can instead pick a position uniformly in their intersection.
    If we can also lower bound the radius of O, then we can first erode C by that distance
    minus that maximum offset distance.
    """
    for obj in scenario.objects:
        # Extract the base region and container region, while doing minor checks.
        base, _, maxDistance, offset = matchInRegion(obj.position)

        if base is None or needsSampling(base):
            continue

        if isinstance(base, regions.EmptyRegion):
            raise InvalidScenarioError(f"Object {obj} placed in empty region")

        container = scenario.containerOfObject(obj)

        if container is None or needsSampling(container):
            continue

        if isinstance(container, regions.EmptyRegion):
            raise InvalidScenarioError(f"Object {obj} contained in empty region")

        # Erode the container region if possible.
        minRadius, _ = supportInterval(obj.inradius)

        if (
            hasattr(container, "buffer")
            and maxDistance is not None
            and minRadius is not None
        ):
            maxErosion = minRadius - maxDistance
            if maxErosion > 0:
                container = container.buffer(-maxErosion)

        # Restrict the base region to the container, unless
        # they're the same in which case we're done
        if base is container:
            continue

        newBase = base.intersect(container)
        newBase.orientation = base.orientation

        # Check if base was a volume and newBase is a surface,
        # in which case the mesh operation might be undefined and we abort.
        if isinstance(base, MeshVolumeRegion) and isinstance(newBase, MeshSurfaceRegion):
            continue

        if isinstance(newBase, EmptyRegion):
            raise InvalidScenarioError(f"Object {obj} does not fit in container")

        if verbosity >= 1:
            if (
                base.dimensionality is None
                or newBase.dimensionality is None
                or base.dimensionality != newBase.dimensionality
            ):
                print(
                    f"    Region containment constraint pruning attempted but could not compute percentage for {base} and {newBase}."
                )
            elif base.dimensionality == newBase.dimensionality:
                ratio = newBase.size / base.size
                percent = max(0, 100 * (1.0 - ratio))

                if percent <= 0.001:
                    # We didn't really prune anything, don't bother setting new position
                    continue

                print(
                    f"    Region containment constraint pruned {percent:.1f}% of space."
                )

        newPos = regions.Region.uniformPointIn(newBase)

        if offset is not None:
            newPos += offset

        obj.position.conditionTo(newPos)


## Pruning based on orientation


def pruneRelativeHeading(scenario, verbosity):
    """Prune based on requirements bounding the relative heading of an Object.

    Specifically, if an object O is:

        * positioned uniformly within a polygonal region B;
        * aligned to a polygonal vector field F (up to a bounded offset);

    and another object O' is:

        * aligned to a polygonal vector field F' (up to a bounded offset);
        * at most some finite maximum distance from O;
        * required to have relative heading within a bounded offset of that of O;

    then we can instead position O uniformly in the subset of B intersecting the cells
    of F which satisfy the relative heading requirements w.r.t. some cell of F' which
    is within the distance bound.
    """
    # TODO Add test for empty pruned polygon (Might cause crash?)
    # Check which objects are (approximately) aligned to polygonal vector fields
    fields = {}
    for obj in scenario.objects:
        field, offsetL, offsetR = matchPolygonalField(obj.heading, obj.position)
        if field is not None:
            fields[obj] = (field, offsetL, offsetR)

    # Check for relative heading relations among such objects
    for obj, (field, offsetL, offsetR) in fields.items():
        position = currentPropValue(obj, "position")
        base, _, _, offset = matchInRegion(position)

        # obj must be positioned uniformly in a Region
        if base is None or needsSampling(base):
            continue

        if offset is not None:
            continue

        basePoly = regions.toPolygon(base)
        if basePoly is None:  # the Region must be polygonal
            continue

        newBasePoly = basePoly
        for rel in obj._relations:
            if isinstance(rel, RelativeHeadingRelation) and rel.target in fields:
                tField, tOffsetL, tOffsetR = fields[rel.target]
                maxDist = maxDistanceBetween(scenario, obj, rel.target)
                if maxDist == float("inf"):
                    # the distance between the objects must be bounded
                    continue
                feasible = feasibleRHPolygon(
                    field,
                    offsetL,
                    offsetR,
                    tField,
                    tOffsetL,
                    tOffsetR,
                    rel.lower,
                    rel.upper,
                    maxDist,
                )
                if feasible is None:
                    # the RH bounds may be too weak to restrict the space
                    continue
                try:
                    pruned = newBasePoly & feasible
                except shapely.geos.TopologicalError:  # TODO how can we prevent these??
                    pruned = newBasePoly & feasible.buffer(0.1, cap_style=2)
                if verbosity >= 1:
                    percent = 100 * (1.0 - (pruned.area / newBasePoly.area))
                    print(
                        f"    Relative heading constraint pruned {percent:.1f}% of space."
                    )
                newBasePoly = pruned

        if newBasePoly is not basePoly:
            newBase = regions.PolygonalRegion(
                polygon=newBasePoly, orientation=base.orientation
            )
            newPos = regions.Region.uniformPointIn(newBase)
            obj.position.conditionTo(newPos)


def maxDistanceBetween(scenario, obj, target):
    """Upper bound the distance between the given Objects."""
    # visDist is initialized to infinity. Then we can use
    # various visibility constraints to upper bound it,
    # keeping the tightest bound.
    ego = scenario.egoObject
    visDist = float("inf")

    if obj is ego and target.requireVisible:
        visDist = min(visDist, visibilityBound(ego, target))
    if target is ego and obj.requireVisible:
        visDist = min(visDist, visibilityBound(ego, obj))
    if obj._observingEntity is target:
        visDist = min(visDist, visibilityBound(target, obj))
    if target._observingEntity is obj:
        visDist = min(visDist, visibilityBound(obj, target))

    # Check for any distance bounds implied by user-specified requirements
    reqDist = float("inf")
    for rel in obj._relations:
        if isinstance(rel, DistanceRelation) and rel.target is target:
            if rel.upper < reqDist:
                reqDist = rel.upper

    return min(visDist, reqDist)


def visibilityBound(obj, target):
    """Upper bound the distance from an Object to another it can see."""
    # Upper bound on visible distance is a sum of several terms:
    # 1. obj.visibleDistance
    _, maxVisibleDistance = supportInterval(obj.visibleDistance)
    if maxVisibleDistance is None:
        return None
    # 2. distance from obj's center to its camera
    _, maxCameraX = supportInterval(obj.cameraOffset.x)
    _, maxCameraY = supportInterval(obj.cameraOffset.y)
    if maxCameraX is None or maxCameraY is None:
        return None
    maxVisibleDistance += math.hypot(maxCameraX, maxCameraY)
    # 3. radius of target
    _, maxRadius = supportInterval(target.radius)
    if maxRadius is None:
        return None
    maxVisibleDistance += maxRadius
    return maxVisibleDistance


def feasibleRHPolygon(
    field, offsetL, offsetR, tField, tOffsetL, tOffsetR, lowerBound, upperBound, maxDist
):
    """Find where objects aligned to the given fields can satisfy the given RH bounds."""
    if (
        offsetR - offsetL >= math.tau
        or tOffsetR - tOffsetL >= math.tau
        or upperBound - lowerBound >= math.tau
    ):
        return None
    polygons = []
    expanded = [(poly.buffer(maxDist), heading) for poly, heading in tField.cells]
    for baseCell, baseHeading in field.cells:
        # TODO skip cells not contained in base region?
        for expandedTargetCell, targetHeading in expanded:
            lower, upper = relativeHeadingRange(
                baseHeading, offsetL, offsetR, targetHeading, tOffsetL, tOffsetR
            )
            if upper >= lowerBound and lower <= upperBound:  # RH intervals overlap
                intersection = baseCell & expandedTargetCell
                if not intersection.is_empty:
                    assert isinstance(
                        intersection, shapely.geometry.Polygon
                    ), intersection
                    polygons.append(intersection)
    return polygonUnion(polygons)


def relativeHeadingRange(
    baseHeading, offsetL, offsetR, targetHeading, tOffsetL, tOffsetR
):
    """Lower/upper bound the possible RH between two headings with bounded disturbances."""
    if baseHeading is None or targetHeading is None:
        # heading may not be constant within cell
        return -math.pi, math.pi
    lower = normalizeAngle(baseHeading + offsetL)
    upper = normalizeAngle(baseHeading + offsetR)
    points = [lower, upper]
    if upper < lower:
        points.extend((math.pi, -math.pi))
    tLower = normalizeAngle(targetHeading + tOffsetL)
    tUpper = normalizeAngle(targetHeading + tOffsetR)
    tPoints = [tLower, tUpper]
    if tUpper < tLower:
        tPoints.extend((math.pi, -math.pi))
    rhs = [tp - p for tp in tPoints for p in points]  # TODO improve
    return min(rhs), max(rhs)
