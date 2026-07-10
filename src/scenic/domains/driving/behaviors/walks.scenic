## Pedestrian Behaviors
def _getBugPath(self, target, backgroundObjects, bufferConst=1):
    """ Generate a walking path using a Bug algorithm approach."""
    # Compute the buffer amount based on our bounding radius and bufferConst
    buffer_amount = shapely.minimum_bounding_radius(self._boundingPolygon) + bufferConst

    # Generate the raw path, from the current position straight towards the target.
    path = LineString([toShapely(self.position), toShapely(toVector(target))])

    # Compute the obstacle polygons
    obstacle_multi_poly = shapely.union_all(list(obj._boundingPolygon.buffer(buffer_amount) for obj in backgroundObjects))
    if isinstance(obstacle_multi_poly, MultiPolygon):
        obstacle_polys = obstacle_multi_poly.geoms
        assert all(isinstance(geom, Polygon) for geom in obstacle_polys)
    elif isinstance(obstacle_multi_poly, Polygon):
        obstacle_polys = [obstacle_multi_poly]
    else:
        assert False

    # Refine path around obstacles, going from those with the largest boundary inwards
    # (to account for the rare case where an obstacle poly may be entirely contained in another)
    for obstacle_poly in sorted(obstacle_polys, key=lambda x: x.boundary.length, reverse=True):
        self_pt = ShapelyPoint(self.position)
        target_pt = ShapelyPoint(path.coords[-1])
        if obstacle_poly.contains(target_pt):
            # Check if target is inside the polygon.
            # If we're too close to the exterior point, return None.
            if obstacle_poly.distance(self_pt) < 0.01:
                return None

            # Otherwise, truncate the path to the closest point on the exterior of the obstacle_poly.
            exterior_intersection = path.intersection(obstacle_poly.exterior)
            stop_pt = shapely.ops.nearest_points(exterior_intersection, self_pt)[0]
            path = shapely.ops.substring(path, 0, path.project(stop_pt, normalized=True), normalized=True)
            continue

        if path.intersects(obstacle_poly):
            # Find intersection points of path with exterior, and extract the first and
            # last with respect to their distance along the path.
            exterior_intersection = path.intersection(obstacle_poly.exterior)
            intersection_points = []

            # If we're inside the obstacle poly, add the closest exterior point to guide us out.
            if obstacle_poly.contains(self_pt):
                intersection_points.append(path.interpolate(path.project(self_pt)))

            if isinstance(exterior_intersection, ShapelyPoint):
                assert obstacle_poly.contains(self_pt)                
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

            intersection_points.sort(key=lambda x: path.project(x))
            start_pt = intersection_points[0]
            end_pt = intersection_points[-1]

            # Split the exterior ring into two segments at these points
            # split_line = shapely.affinity.scale(LineString([start_pt, end_pt]), 2, 2)
            exterior_ls = LineString(obstacle_poly.exterior)
            start_pt_s = exterior_ls.project(start_pt, normalized=True)
            exterior_ls = LineString(list(shapely.ops.substring(exterior_ls, start_pt_s, 1, normalized=True).coords)
                                     + list(shapely.ops.substring(exterior_ls, 0, start_pt_s, normalized=True).coords))
            end_pt_ls = exterior_ls.project(end_pt, normalized=True)
            exterior_segments = [
                shapely.ops.substring(exterior_ls, 0, end_pt_ls, normalized=True),
                shapely.ops.substring(exterior_ls, end_pt_ls, 1, normalized=True)
            ]

            # Patch together the shorter of these exterior segments with the path.
            start_path = shapely.ops.substring(path, 0, path.project(start_pt, normalized=True), normalized=True)
            end_path = shapely.ops.substring(path, path.project(end_pt, normalized=True), 1, normalized=True)
            start_path = shapely.force_2d(start_path)
            end_path = shapely.force_2d(end_path)

            # Extract and reverse mid_path if needed
            mid_path = sorted(exterior_segments, key=lambda x: x.length)[0]
            mid_path_start = ShapelyPoint(mid_path.coords[0])
            if (ShapelyPoint(mid_path.coords[0]).distance(ShapelyPoint(start_path.coords[0]))
                > ShapelyPoint(mid_path.coords[0]).distance(ShapelyPoint(end_path.coords[0]))):
                mid_path = mid_path.reverse()

            path = LineString(list(start_path.coords) + list(mid_path.coords) + list(end_path.coords))

    return path

behavior WalkPath(path, targetSpeed, terminationThresh=0.1, replanTime=0.5):
    """ Walk a path at targetSpeed, stopping at the end."""
    if not isinstance(path, PolylineRegion):
        raise ValueError("`path` must be a `PolylineRegion`.")
    path = path.lineString

    while distance from self to target > doneThresh:
        # Find where the actor currently is on the path
        start_s = path.project(ShapelyPoint(self.position))
        path_distance = path.interpolate(start_s).distance(ShapelyPoint(self.position))
        
        # Compute lookahead distance from speed and timestep. This accounts for how
        # far we are from the path as well, so that we prioritize returning to it.
        lookahead_dist = max(targetSpeed * simulation().timestep - path_distance, 0)

        # Find target point
        target_point = Vector(*path.interpolate(start_s+lookahead_dist).coords[0])
        
        # Set appropriate heading and velocity, calculating actual speed we should aim
        # for, so we don't overshoot if we cut a corner or are at the end of the path.
        actual_speed = min(targetSpeed, (distance from self to target_point)/simulation().timestep)
        heading = angle from self to target_point
        take SetWalkingDirectionAction(heading), SetWalkingSpeedAction(actual_speed)
    
    take SetWalkingSpeedAction(0)

behavior WalkTo(target, targetSpeed, terminationThresh=0.1, replanTime=0.5):
    """ Walk towards a given target position at targetSpeed, stopping at the end."""
    path = PolylineRegion(points=(self.position, targetSpeed))
    do WalkPath(path)

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
