import collections
import math
import shapely
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon, MultiLineString, LineString, MultiPoint, Point as ShapelyPoint

import scenic.domains.driving.model as _model
from scenic.core.regions import toShapely
from scenic.core.type_support import toVector
from scenic.domains.driving.actions import *

## Pedestrian Behaviors
def getBugPath(actor, path_ls, backgroundObjects, additionalPolys, bufferCalc):
    """ Refine a walking path using a Bug algorithm approach."""
    assert isinstance(path_ls, LineString)

    # Lambda to compute buffer const.
    baseBuffer = shapely.minimum_bounding_radius(actor._boundingPolygon)

    # Compute the obstacle polygons, with some forward prediction.
    obst_polys = [(obj, obj._boundingPolygon.buffer(bufferCalc(obj))) for obj in backgroundObjects]
    obst_polys += [(obj, shapely.transform(poly, lambda x: x + (t*obj.velocity.x, t*obj.velocity.y)))
                 for t in [0.5, 1] for obj, poly in obst_polys if not obj.isVehicle]

    # Only track non-vehicles as historicaly polys, as those are the only ones we will
    # path around while moving. Vehicles are expected to yield to us.
    hist_multi_poly = shapely.union_all([poly for obj, poly in obst_polys if not obj.isVehicle])

    obst_multi_poly = shapely.union_all([p for _,p in obst_polys] + list(additionalPolys))

    if isinstance(obst_multi_poly, MultiPolygon):
        obst_polys = obst_multi_poly.geoms
        assert all(isinstance(geom, Polygon) for geom in obst_polys)
    elif isinstance(obst_multi_poly, Polygon):
        obst_polys = [obst_multi_poly]
    elif obst_multi_poly.is_empty:
        obst_polys = []
    else:
        assert False

    # Refine path around obstacles, going from those with the largest boundary inwards
    # (to account for the rare case where an obstacle poly may be entirely contained in another)
    for obstacle_poly in sorted(obst_polys, key=lambda x: x.boundary.length, reverse=True):
        self_pt = ShapelyPoint(actor.position)
        target_pt = ShapelyPoint(path_ls.coords[-1])
        if obstacle_poly.contains(target_pt):
            # Check if target is inside the polygon.
            # If we're too close to the exterior point, return None.
            if obstacle_poly.distance(self_pt) < 0.01:
                return None, hist_multi_poly

            # Otherwise, truncate the path to the closest point on the exterior of the obstacle_poly.
            exterior_intersection = path_ls.intersection(obstacle_poly.exterior)
            stop_pt = shapely.ops.nearest_points(exterior_intersection, self_pt)[0]
            path_ls = shapely.ops.substring(path_ls, 0, path_ls.project(stop_pt, normalized=True), normalized=True)
            continue

        if path_ls.intersects(obstacle_poly):
            # Find intersection points of path with exterior, and extract the first and
            # last with respect to their distance along the path.
            exterior_intersection = path_ls.intersection(obstacle_poly.exterior)
            intersection_points = []

            # If we're inside the obstacle poly, add the closest exterior point to guide us out.
            if obstacle_poly.contains(self_pt):
                intersection_points.append(path_ls.interpolate(path_ls.project(self_pt)))

            if isinstance(exterior_intersection, ShapelyPoint):
                assert obstacle_poly.contains(ShapelyPoint(path_ls.coords[0]))                
                intersection_points.append(exterior_intersection)
            elif isinstance(exterior_intersection, LineString):
                instersection_points += [ShapelyPoint(geom.coords[0]), ShapelyPoint(geom,coords[1])]
            elif isinstance(exterior_intersection, (MultiPoint, MultiLineString, GeometryCollection)):
                for geom in exterior_intersection.geoms:
                    if isinstance(geom, ShapelyPoint):
                        intersection_points.append(geom)
                    elif isinstance(geom, LineString):
                        instersection_points += [ShapelyPoint(geom.coords[0]), ShapelyPoint(geom,coords[1])]
            else:
                assert False

            intersection_points.sort(key=lambda x: path_ls.project(x))
            start_pt = intersection_points[0]
            end_pt = intersection_points[-1]

            # Split the exterior ring into two segments at these points
            exterior_ls = LineString(obstacle_poly.exterior)
            start_pt_s = exterior_ls.project(start_pt, normalized=True)
            exterior_ls = LineString(list(shapely.ops.substring(exterior_ls, start_pt_s, 1, normalized=True).coords)
                                     + list(shapely.ops.substring(exterior_ls, 0, start_pt_s, normalized=True).coords))
            exterior_ls = shapely.remove_repeated_points(exterior_ls)
            end_pt_ls = exterior_ls.project(end_pt, normalized=True)
            exterior_segments = [
                shapely.ops.substring(exterior_ls, 0, end_pt_ls, normalized=True),
                shapely.ops.substring(exterior_ls, end_pt_ls, 1, normalized=True)
            ]

            # Patch together the shorter of these exterior segments with the path.
            start_path = shapely.ops.substring(path_ls, 0, path_ls.project(start_pt, normalized=True), normalized=True)
            end_path = shapely.ops.substring(path_ls, path_ls.project(end_pt, normalized=True), 1, normalized=True)
            start_path = shapely.force_2d(start_path)
            end_path = shapely.force_2d(end_path)

            # Extract and reverse mid_path (if needed). If paths are very close in length,
            # bias to the right.
            # TODO: Bias to the appropriate driving direction
            if 0.9 < exterior_segments[0].length/exterior_segments[1].length < 1.1:
                def angle_helper(ls):
                    return actor.apparentHeadingTo(Vector(*ls.centroid.coords[0]))
                exterior_segments.sort(key=lambda x: angle_helper(x))
            else:
                exterior_segments.sort(key=lambda x: x.length)
            mid_path = exterior_segments[0]
            mid_path_start = ShapelyPoint(mid_path.coords[0])
            if (ShapelyPoint(mid_path.coords[0]).distance(ShapelyPoint(start_path.coords[0]))
                > ShapelyPoint(mid_path.coords[0]).distance(ShapelyPoint(end_path.coords[0]))):
                mid_path = mid_path.reverse()

            path_ls = LineString(list(start_path.coords) + list(mid_path.coords) + list(end_path.coords))
            path_ls = shapely.remove_repeated_points(path_ls)

    import matplotlib.pyplot as plt
    plt.gca().set_aspect("equal")
    from scenic.core.geometry import plotPolygon
    from scenic.syntax.veneer import simulation
    if actor.name == "pedA" and simulation().currentRealTime == int(simulation().currentRealTime) and simulation().currentRealTime > 0:
        simulation().scene.workspace.network.show()
        for obj in simulation().objects:
            obj.show2D(simulation().scene.workspace, plt)
        simulation().scene.workspace.zoomAround(plt, simulation().objects)
        plotPolygon(obst_multi_poly, plt, style="c--")
        plt.show()

    return path_ls, hist_multi_poly

behavior TieBreakingPause():
    take SetWalkingSpeedAction(0)
    wait for Range(0.1, 0.5) seconds

behavior _WalkPathHelper(path, targetSpeed):
    dist_along = 0
    while True:
        # Determine target point, which will move along the path until we re-plan.
        dist_along += targetSpeed*simulation().timestep
        target_pt = Vector(*path.interpolate(dist_along).coords[0])
        # Set appropriate heading and velocity, calculating actual speed we should aim
        # for, so we don't overshoot if we cut a corner or are at the end of the path.
        actual_speed = min(targetSpeed, (distance from self to target_pt)/simulation().timestep)
        heading = angle from self to target_pt
        take SetWalkingDirectionAction(heading), SetWalkingSpeedAction(actual_speed)

behavior WalkPath(path, targetSpeed, *, avoidObstacles=True,
    terminationThresh=0.1, replanTime=0.5, obstHistory=4,
    vehBuffer=1, nonVehBuffer=0.25, erosionFactor=1):
    """ Walk a path at targetSpeed, stopping at the end."""
    if not isinstance(path, PolylineRegion):
        raise ValueError("`path` must be a `PolylineRegion`.")
    path_ls = path.lineString
    obstacle_poly_hist = []

    bufferCalc = lambda obj: (shapely.minimum_bounding_radius(self._boundingPolygon)
        + (vehBuffer if obj.isVehicle else nonVehBuffer))

    while distance from self to path.end > terminationThresh:
        # Refine the path by dropping already traversed areas and adding a link to the start.
        start_s = path_ls.project(ShapelyPoint(self.position), normalized=True)
        refined_path_ls = shapely.ops.substring(path_ls, start_s, 1, normalized=True)
        refined_path_ls =  LineString([self.position] + list(refined_path_ls.coords))
        refined_path_ls = shapely.remove_repeated_points(refined_path_ls)

        # If our immediate path has us cross through any objects in motion, stop and 
        # wait until it's clear.
        background_objects = [obj for obj in simulation().objects if obj is not self]
        immediate_path = shapely.ops.substring(refined_path_ls, 0, targetSpeed)
        danger_objects = [obj for obj in background_objects if obj.speed > 0.1 and obj.isVehicle]
        moving_obj_danger_zone = shapely.union_all(
            [obj._boundingPolygon.buffer(bufferCalc(obj)) for obj in danger_objects]
        )
        if immediate_path.intersects(moving_obj_danger_zone):
            do TieBreakingPause()
            continue

        # Modify path to route around objects.
        refined_path_ls, poly = getBugPath(self, refined_path_ls, background_objects, obstacle_poly_hist, bufferCalc)
        
        # Update and slightly erode the obstacle poly history
        stepErosionFactor = min(replanTime/obstHistory, 1) * erosionFactor
        obstacle_poly_hist = [p.buffer(-stepErosionFactor) for p in obstacle_poly_hist] + [poly]
        obstacle_poly_hist = obstacle_poly_hist[-math.ceil(obstHistory/replanTime):]

        # If refined_path_ls is None, our goal is inside the danger zone and we can't
        # proceed further right now.
        if refined_path_ls is None:
            do TieBreakingPause()
            continue

        # Follow the path until we replan, terminating early if we reach the end.
        try:
            do _WalkPathHelper(refined_path_ls, targetSpeed) for replanTime seconds
        interrupt when distance from self to Vector(*refined_path_ls.coords[-1]) < terminationThresh:
            abort

    take SetWalkingSpeedAction(0)

behavior WalkTo(target, targetSpeed, *, avoidObstacles=True):
    """ Walk towards a given target position at targetSpeed, stopping at the end."""
    path = PolylineRegion(points=(self.position, toVector(target)))
    do WalkPath(path, targetSpeed, avoidObstacles=avoidObstacles)

behavior Walk(targetSpeed=None, backwards=None, avoidObstacles=True):
    if targetSpeed is None:
        # TODO: Should we move this to a property of pedestrians? (`baseWalkSpeed`?)
        targetSpeed = Range(0.9, 1.8) # From ~2mph to ~4mph

    if backwards is None:
        backwards = Uniform(True, False)

    network = _model.network

    # TODO: Have pedestrians bias towards the appropriate side of the sidewalk based off road direction?
    while True:
        # If we're not currently in a walkable region, return to the closest one.
        if self.position not in network.walkableRegion:
            # TODO: Replace with closest point in region operator.
            closest_pt = shapely.ops.nearest_points(toShapely(network.walkableRegion), toShapely(self.position))[0]
            target_element = network.findPointIn(Vector(*closest_pt.coords[0]), network.sidewalks+network.crossings, reject=False)
            target_pt = target_element.centerline.project(self.position)
            do WalkTo(target_pt, targetSpeed=targetSpeed, avoidObstacles=avoidObstacles)
            continue
        
        # If we're not close to the start or end of the centerline of our current element
        # (depending on whether we are walking `backwards` or not), walk towards it following the centerline.
        current_element = network.findPointIn(self.position, network.sidewalks+network.crossings, reject=False)
        end_pt = current_element.centerline.start if backwards else current_element.centerline.end
        if distance from self.position to end_pt > 0.1:
            target_path = current_element.centerline.reverse() if backwards else current_element.centerline
            do WalkPath(target_path, targetSpeed=targetSpeed, avoidObstacles=avoidObstacles)
            continue
        
        # If we're at the end of the current element, we should pick a successor/predecessor
        # (depending on whether we are walking `backwards`).
        # TODO: Randomly pick from ALL successors/predecessors and sidewalks.
        next_element = current_element._predecessor if backwards else current_element._successor
        if next_element is not None:
            target_path = next_element.centerline.reverse() if backwards else next_element.centerline
            do WalkPath(target_path, targetSpeed=targetSpeed, avoidObstacles=avoidObstacles)
            continue

        # We have no valid next moves. Terminate the behavior.
        return

# TODO: This uses WalkTowardsAction which doesn't exist.
behavior WalkForwardBehavior():
    """Walk forward behavior for pedestrians.

    It will uniformly randomly choose either end of the sidewalk that
    the pedestrian is on, and have the pedestrian walk towards the endpoint.
    """
    current_sidewalk = _model.network.sidewalkAt(self.position)
    end_point = Uniform(*current_sidewalk.centerline.points)
    end_vec = end_point[0] @ end_point[1]
    normal_vec = Vector.normalized(end_vec)
    take WalkTowardsAction(goal_position=normal_vec), SetSpeedAction(speed=1)
