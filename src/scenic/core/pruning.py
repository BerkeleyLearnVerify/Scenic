
import math
import time
import shapely.geometry
import shapely.geos

from scenic.core.distributions import Samplable, MethodDistribution, OperatorDistribution
from scenic.core.distributions import needsSampling, supportInterval, underlyingFunction
from scenic.core.object_types import Point, Object
from scenic.core.geometry import normalizeAngle, polygonUnion, plotPolygon
from scenic.core.vectors import VectorField, PolygonalVectorField, VectorMethodDistribution
from scenic.core.workspaces import Workspace
from scenic.syntax.relations import RelativeHeadingRelation
import scenic.core.regions as regions

def currentPropValue(obj, prop):
    value = getattr(obj, prop)
    return value._conditioned if isinstance(value, Samplable) else value

def isMethodCall(thing, method):
    if not isinstance(thing, (MethodDistribution, VectorMethodDistribution)):
        return False
    return thing.method is underlyingFunction(method)

def matchInRegion(position):
    position = position.toVector()
    if isinstance(position, regions.PointInRegionDistribution):
        reg = position.region
        if isinstance(reg, Workspace):
            reg = reg.region
        return reg
    return None

def matchPolygonalField(heading, position):
    if (isMethodCall(heading, VectorField.__getitem__)
        and isinstance(heading.object, PolygonalVectorField)
        and heading.arguments == (position.toVector(),)):
        return heading.object, 0, 0
    elif isinstance(heading, OperatorDistribution) and heading.operator in ('__add__', '__radd__'):
        field, lower, upper = matchPolygonalField(heading.object, position)
        if field is not None:
            assert len(heading.operands) == 1
            offset = heading.operands[0]
            ol, oh = supportInterval(offset)
            if ol is not None and oh is not None:
                return field, lower + ol, upper + oh
    return None, 0, 0

def prune(scenario, verbosity=1):
    if verbosity >= 1:
        print('  Pruning scenario...')
        startTime = time.time()

    pruneContainment(scenario, verbosity)
    pruneRelativeHeading(scenario, verbosity)

    if verbosity >= 1:
        totalTime = time.time() - startTime
        print(f'  Pruned scenario in {totalTime:.4g} seconds.')

def pruneContainment(scenario, verbosity):
    """Prune based on the requirement that individual Objects fit within their container.

    Specifically, if O is positioned uniformly in region B and has container C, then we
    can instead pick a position uniformly in their intersection. If we can also lower
    bound the radius of O, then we can first erode C by that distance.
    """
    for obj in scenario.objects:
        base = matchInRegion(obj.position)
        if base is None:
            continue
        basePoly = regions.toPolygon(base)
        if basePoly is None:
            continue
        container = scenario.containerOfObject(obj)
        containerPoly = regions.toPolygon(container)
        if containerPoly is None:
            return None
        minRadius, _ = supportInterval(obj.inradius)
        if minRadius is not None:
            containerPoly = containerPoly.buffer(-minRadius)
        elif base is container:
            continue
        newBasePoly = basePoly & containerPoly
        if verbosity >= 1:
            percent = 100 * (1.0 - (newBasePoly.area / basePoly.area))
            print(f'    Region containment constraint pruned {percent:.1f}% of space.')
        newBase = regions.PolygonalRegion(polygon=newBasePoly,
                                          orientation=base.orientation)
        newPos = regions.Region.uniformPointIn(newBase)
        obj.position.conditionTo(newPos)

def pruneRelativeHeading(scenario, verbosity):
    # Check which objects are (approximately) aligned to polygonal vector fields
    fields = {}
    for obj in scenario.objects:
        field, offsetL, offsetR = matchPolygonalField(obj.heading, obj.position)
        if field is not None:
            fields[obj] = (field, offsetL, offsetR)
    # Check for relative heading relations
    for obj, (field, offsetL, offsetR) in fields.items():
        position = currentPropValue(obj, 'position')
        base = matchInRegion(position)
        if base is None:
            continue
        basePoly = regions.toPolygon(base)
        if basePoly is None:
            continue
        newBasePoly = basePoly
        for rel in obj._relations:
            if isinstance(rel, RelativeHeadingRelation) and rel.target in fields:
                tField, tOffsetL, tOffsetR = fields[rel.target]
                maxDist = maxDistanceBetween(scenario, obj, rel.target)
                if maxDist is None:
                    continue
                feasible = feasibleRHPolygon(field, offsetL, offsetR,
                                             tField, tOffsetL, tOffsetR,
                                             rel.lower, rel.upper, maxDist)
                if feasible is None:
                    continue
                try:
                    pruned = newBasePoly & feasible
                except shapely.geos.TopologicalError:   # TODO how can we prevent these??
                    pruned = newBasePoly & feasible.buffer(0.1, cap_style=2)
                if verbosity >= 1:
                    percent = 100 * (1.0 - (pruned.area / newBasePoly.area))
                    print(f'    Relative heading constraint pruned {percent:.1f}% of space.')
                newBasePoly = pruned
        if newBasePoly is not basePoly:
            newBase = regions.PolygonalRegion(polygon=newBasePoly,
                                              orientation=base.orientation)
            newPos = regions.Region.uniformPointIn(newBase)
            obj.position.conditionTo(newPos)

def maxDistanceBetween(scenario, obj, target):
    # TODO also infer from requirements
    ego = scenario.egoObject
    if not ((obj is ego and target.requireVisible)
            or (target is ego and obj.requireVisible)):
        return None
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

def feasibleRHPolygon(field, offsetL, offsetR,
                      tField, tOffsetL, tOffsetR,
                      lowerBound, upperBound, maxDist):
    if (offsetR - offsetL >= math.tau
        or tOffsetR - tOffsetL >= math.tau
        or upperBound - lowerBound >= math.tau):
        return None
    polygons = []
    expanded = [(poly.buffer(maxDist), heading) for poly, heading in tField.cells]
    for baseCell, baseHeading in field.cells:
        for expandedTargetCell, targetHeading in expanded:
            lower, upper = relativeHeadingRange(baseHeading, offsetL, offsetR,
                                                targetHeading, tOffsetL, tOffsetR)
            if (upper >= lowerBound and lower <= upperBound):   # RH intervals overlap
                intersection = baseCell & expandedTargetCell
                if not intersection.is_empty:
                    assert isinstance(intersection, shapely.geometry.Polygon), intersection
                    polygons.append(intersection)
    return polygonUnion(polygons)

def relativeHeadingRange(baseHeading, offsetL, offsetR,
                         targetHeading, tOffsetL, tOffsetR):
    if baseHeading is None or targetHeading is None:    # heading may not be constant within cell
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
    rhs = [tp - p for tp in tPoints for p in points]    # TODO improve
    return min(rhs), max(rhs)
