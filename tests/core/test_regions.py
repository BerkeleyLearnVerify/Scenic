import math
from pathlib import Path


import pytest
import math
import shapely.geometry

from scenic.core.object_types import Object, OrientedPoint
from scenic.core.regions import *
from scenic.core.errors import RuntimeParseError
from scenic.core.vectors import VectorField
from tests.utils import sampleSceneFrom


def test_all_region():
    ar = AllRegion("all")
    assert ar in {ar}  # check hashability
    assert ar == everywhere
    assert ar.containsPoint(Vector(347, -962.5))
    circ = CircularRegion(Vector(29, 34), 5)
    assert ar.intersect(circ) is circ
    assert circ.intersect(ar) is circ
    assert ar.intersects(circ)
    assert circ.intersects(ar)
    diff = ar.difference(circ)
    assert diff.containsPoint(Vector(0, 0))
    assert not diff.containsPoint(Vector(29, 34))
    assert ar.containsObject(Object._with())
    assert circ.difference(ar) == nowhere
    assert ar.union(circ) == ar
    assert circ.union(ar) == ar
    assert ar.distanceTo(Vector(4, 12)) == 0


def test_empty_region():
    er = EmptyRegion("none")
    assert er in {er}
    assert er == nowhere
    assert not er.containsPoint(Vector(0, 0))
    assert not er.containsObject(Object._with())
    circ = CircularRegion(Vector(29, 34), 5)
    assert er.intersect(circ) == er
    assert circ.intersect(er) == er
    assert not er.intersects(circ)
    assert not circ.intersects(er)
    assert circ.difference(er) is circ
    assert er.difference(circ) == er
    assert er.union(circ) is circ
    assert circ.union(er) is circ
    assert er.distanceTo(Vector(4, 12)) == float("inf")


def test_circular_region():
    circ = CircularRegion(Vector(4, -3), 2)
    assert circ in {circ}
    for pt in ((4, -3), (6, -3), (2, -3), (4, -5), (4, -1), (5, -2)):
        assert circ.containsPoint(Vector(*pt))
    for pt in ((6, -1), (6, -5), (2, -5), (2, -1), (6.1, -3)):
        assert not circ.containsPoint(Vector(*pt))
    circ2 = CircularRegion(Vector(1, -7), 3.1)
    assert circ.intersects(circ2)
    assert circ2.intersects(circ)
    circ3 = CircularRegion(Vector(1, -7), 2.9)
    assert not circ.intersects(circ3)
    assert not circ3.intersects(circ)
    assert circ.distanceTo(Vector(4, -3)) == 0
    assert circ.distanceTo(Vector(1, -7)) == pytest.approx(3)
    assert circ.getAABB() == ((2, -5), (6, -1), (0, 0))


def test_circular_sampling():
    center = Vector(4, -3)
    circ = CircularRegion(center, 2)
    pts = [circ.uniformPointInner() for i in range(3000)]
    dists = [pt.distanceTo(center) for pt in pts]
    assert all(dist <= 2 for dist in dists)
    assert sum(dist <= 1.414 for dist in dists) >= 1250
    assert sum(dist > 1.414 for dist in dists) >= 1250
    xs, ys, zs = zip(*pts)
    assert sum(x >= 4 for x in xs) >= 1250
    assert sum(y >= -3 for y in ys) >= 1250


def test_rectangular_region():
    rect = RectangularRegion(Vector(1, 2), math.radians(30), 4, 2)
    assert rect in {rect}
    for pt in [(1, 2), (2.2, 3.8), (3.1, 2.15), (-1.1, 1.85), (1, 3.1)]:
        assert rect.containsPoint(pt)
    for pt in [(1, 3.5), (2.8, 3), (-0.5, 2.3)]:
        assert not rect.containsPoint(pt)
    r2 = RectangularRegion(Vector(3.5, 2.5), 0, 1, 1)
    assert rect.intersects(r2)
    r3 = RectangularRegion(Vector(2.5, 4.5), 0, 1, 1)
    assert not rect.intersects(r3)
    assert rect.distanceTo((1 + 2 * math.sqrt(3), 4)) == pytest.approx(2)
    (minx, miny), (maxx, maxy), _ = rect.getAABB()
    assert maxy == pytest.approx(3 + math.sqrt(3) / 2)
    assert miny == pytest.approx(1 - math.sqrt(3) / 2)
    assert maxx == pytest.approx(1.5 + math.sqrt(3))
    assert minx == pytest.approx(0.5 - math.sqrt(3))


def test_polyline_region():
    # With standard orientation
    pl = PolylineRegion([(0, 2), (1, 1), (0, 0)])
    assert pl in {pl}
    assert len(pl) == 3
    for pt in ((0, 2), (0.5, 1.5), (1, 1), (0.5, 0.5), (0, 0)):
        assert pl.containsPoint(pt)
    for pt in ((1, 2), (1, 0), (0, 1), (0.5, 1), (2, 2), (-1, -1)):
        assert not pl.containsPoint(pt)
    assert not pl.containsObject(Object._with())
    assert pl.project((0, 3)) == (0, 2, 0)
    assert pl.distanceTo((0, 3)) == pytest.approx(1)
    assert pl.project((0.3, 0.2)) == pytest.approx((0.25, 0.25, 0))
    assert pl.distanceTo((0.3, 0.2)) == pytest.approx(0.05 * math.sqrt(2))
    assert pl.pointAlongBy(math.sqrt(2)) == pytest.approx((1, 1, 0))
    assert pl.pointAlongBy(0.5, normalized=True) == pytest.approx((1, 1, 0))
    assert pl.pointAlongBy(0.75, normalized=True) == pytest.approx((0.5, 0.5, 0))
    assert pl.equallySpacedPoints(3) == list(pl.points)
    assert pl.pointsSeparatedBy(math.sqrt(2)) == list(pl.points[:-1])
    assert pl.length == pytest.approx(2 * math.sqrt(2))
    assert pl.getAABB() == ((0, 0), (1, 2), (0, 0))
    start = pl.start
    assert isinstance(start, OrientedPoint)
    assert start.position == (0, 2)
    assert start.heading == pytest.approx(math.radians(-135))
    end = pl.end
    assert isinstance(end, OrientedPoint)
    assert end.position == (0, 0)
    assert end.heading == pytest.approx(math.radians(135))
    # With non-standard orientation
    vf = VectorField("foo", lambda pos: 0.25 + (pos.x + pos.y) / 2)
    pl2 = PolylineRegion([(1, 1), (1, -1), (-1, -1), (-1, 1)], orientation=vf)
    assert not pl2.containsObject(Object._with(width=2, length=2))
    assert pl2.start.heading == pytest.approx(1.25)
    assert pl2.end.heading == pytest.approx(0.25)
    assert pl.intersects(pl2)
    assert pl2.intersects(pl)
    # With no orientation
    pl = PolylineRegion([(0, 0), (-1, 1), (1, 1)], orientation=None)
    assert pl.start.heading == 0
    assert pl.end.heading == 0
    poly = PolygonalRegion([(0, 0.5), (0, 1), (2, 1), (0, 0)])
    assert pl.intersects(poly)
    assert poly.intersects(pl)
    i = pl.intersect(poly)
    assert isinstance(i, PolylineRegion)
    assert set(i.points) == {(0, 1, 0), (1, 1, 0)}
    d = pl.difference(poly)
    assert isinstance(d, PolylineRegion)
    assert set(d.points) == {(0, 0, 0), (-1, 1, 0), (0, 1, 0)}
    assert PolylineRegion.unionAll(()) == nowhere
    pl2 = PolylineRegion([(2, 0), (3, 0)])
    union = PolylineRegion.unionAll([pl, pl2])
    assert isinstance(union, PolylineRegion)
    assert union.points[:3] == pl.points
    assert union.points[3:] == pl2.points
    assert pl + pl2 == union
    pl3 = PolylineRegion([(3, 0), (3, 1)])
    u2 = PolylineRegion.unionAll([union, pl3])
    assert isinstance(u2, PolylineRegion)
    assert u2.points[:5] == union.points
    assert u2.points[4:] == pl3.points
    assert union + pl3 == u2
    assert pl2.intersects(pl3)
    assert pl3.intersects(pl2)
    i = pl2.intersect(pl3)
    assert isinstance(i, PointSetRegion)
    assert i.points.tolist() == [[3, 0, 0]]
    pl4 = PolylineRegion([(3, 1), (3, 0)])
    assert pl3 != pl4
    assert pl3.difference(pl4) == nowhere
    pl5 = PolylineRegion([(2, 1), (4, 0.5), (2, 0)])
    i = pl4.intersect(pl5)
    assert isinstance(i, PointSetRegion)
    assert len(i.points) == 2


def test_polygon_region():
    poly = PolygonalRegion([(1, 1), (3, 1), (2, 2), (1.3, 1.15)])
    assert poly in {poly}
    for pt in [(1, 1), (2, 1), (2, 1.5), (2.5, 1.4)]:
        assert poly.containsPoint(pt)
    for pt in [(1, 1.1), (2, 0.9), (2, 2.1), (2.9, 1.2), (1.5, 1.4)]:
        assert not poly.containsPoint(pt)
    assert poly.containsObject(Object._with(position=(2, 1.25), width=0.49, length=0.49))
    assert not poly.containsObject(Object._with(position=(2, 1.25), width=1, length=0.49))
    assert poly.getAABB() == ((1, 1), (3, 2), (0, 0))
    line = PolylineRegion([(1, 1), (2, 1.8)])
    assert poly.intersects(line)
    assert line.intersects(poly)
    i = poly.intersect(line)
    assert isinstance(i, PolylineRegion)
    poly2 = PolygonalRegion([(1, 1), (2, 1.8), (3, 2), (1, 2)])
    assert poly.intersects(poly2)
    i = poly.intersect(poly2)
    assert isinstance(i, PolygonalRegion)
    assert i.containsPoint((2, 2))
    d = poly.difference(poly2)
    assert isinstance(d, PolygonalRegion)
    assert not d.containsPoint((2, 2))


def test_polygon_sampling():
    p = shapely.geometry.Polygon(
        [(0, 0), (0, 3), (3, 3), (3, 0)], holes=[[(1, 1), (1, 2), (2, 2), (2, 1)]]
    )
    r = PolygonalRegion(polygon=p)
    pts = [r.uniformPointInner() for i in range(3000)]
    for x, y, z in pts:
        assert 0 <= x <= 3 and 0 <= y <= 3 and z == 0
        assert not (1 < x < 2 and 1 < y < 2)
    xs, ys, zs = zip(*pts)
    assert sum(1 <= x <= 2 for x in xs) <= 870
    assert sum(1 <= y <= 2 for y in ys) <= 870
    assert sum(x >= 1.5 for x in xs) >= 1250
    assert sum(y >= 1.5 for y in ys) >= 1250


def test_mesh_region_fromFile():
    MeshVolumeRegion.fromFile(
        Path(".").parent.parent.parent / "tools" / "meshes" / "classic_plane.obj.bz2",
        dimensions=(20, 20, 10),
        rotation=(math.radians(-90), 0, math.radians(-10)),
    )
    MeshSurfaceRegion.fromFile(
        Path(".").parent.parent.parent / "tools" / "meshes" / "classic_plane.obj.bz2",
        dimensions=(20, 20, 10),
        rotation=(math.radians(-90), 0, math.radians(-10)),
    )


def test_mesh_operation_blender():
    r1 = BoxRegion(position=(0, 0, 0), dimensions=(1, 1, 1), engine="blender")
    r2 = BoxRegion(position=(0, 0, 0), dimensions=(2, 2, 2), engine="blender")

    r = r1.intersect(r2)


def test_mesh_operation_scad():
    r1 = BoxRegion(position=(0, 0, 0), dimensions=(1, 1, 1), engine="scad")
    r2 = BoxRegion(position=(0, 0, 0), dimensions=(2, 2, 2), engine="scad")

    r = r1.intersect(r2)


def test_mesh_volume_region_sampling():
    r = BoxRegion(position=(0, 0, 0), dimensions=(2, 2, 2))
    pts = [r.uniformPointInner() for _ in range(100)]

    for x, y, z in pts:
        assert -1 <= x <= 1
        assert -1 <= y <= 1
        assert -1 <= z <= 1


def test_mesh_surface_region_sampling():
    r = BoxRegion(position=(0, 0, 0), dimensions=(2, 2, 2)).getSurfaceRegion()
    pts = [r.uniformPointInner() for _ in range(100)]

    for x, y, z in pts:
        assert x == 1 or x == -1 or y == 1 or y == -1 or z == 1 or z == -1


def test_path_region():
    points_2d = [(0, 0), [0, 1], (1, 1), (1, 0)]
    points_3d = [(3, 3, 0), (3, 4, 1), [4, 4, 2], (4, 3, 3)]
    points_mixed = [(6, 6), (6, 7, 1), (7, 7, 2), [7, 6, 3]]
    polylines = [points_2d, points_3d, points_mixed]

    # Test creation
    r1 = PathRegion(points=points_2d)
    r2 = PathRegion(points=points_3d)
    r3 = PathRegion(points=points_mixed)
    r4 = PathRegion(polylines=polylines)

    with pytest.raises(RuntimeParseError):
        PathRegion(points=polylines)

    with pytest.raises(RuntimeParseError):
        PathRegion(polylines=points_mixed)

    # Test simple containment
    assert r1.containsPoint((0, 0))
    assert r1.containsPoint((0, 1))
    assert r1.containsPoint((0, 0.5))
    assert not r1.containsPoint((0, 0, 1))
    assert not r1.containsPoint((0, -1))

    assert r2.containsPoint((3, 3, 0))
    assert r2.containsPoint((3, 4, 1))
    assert r2.containsPoint((3, 3.5, 0.5))
    assert not r2.containsPoint((3, 3, 1))
    assert not r2.containsPoint((-2, -2, -2))

    # Test sampling and containment of samples
    for _ in range(100):
        target_region = random.choice([r1, r2, r3, r4])
        sampled_pt = target_region.uniformPointInner()
        assert target_region.containsPoint(sampled_pt)


def test_mesh_region_distribution():
    sampleSceneFrom(
        """
        position = (Range(-5,5), Range(-5,5), Range(-5,5))
        radius = Range(1,5)
        dimensions = (2*radius, 2*radius, 2*radius)
        rotation = (Range(0,360), Range(0,360), Range(0,360))

        region = SpheroidRegion(position=position, dimensions=dimensions, rotation=rotation)

        ego = new Object in region
    """,
        maxIterations=100,
    )


def test_mesh_polygon_intersection():
    r1 = BoxRegion(position=(0, 0, 0), dimensions=(3, 3, 2))
    r2 = CircularRegion((0, 0), 1, resolution=64)

    r = r1.intersect(r2)

    assert isinstance(r, PolygonalRegion)

    v_pts = [r.uniformPointInner() for _ in range(100)]

    for x, y, z in v_pts:
        assert math.hypot(x, y) <= 1
        assert z == 0
        assert r1.containsPoint(Vector(x, y, z))
        assert r2.containsPoint(Vector(x, y, z))


def test_mesh_polygons_intersection():
    p1 = shapely.geometry.Polygon([(1, 1), (1, 2), (2, 2), (2, 1)])
    r1 = PolygonalRegion(polygon=p1)

    p2 = shapely.geometry.Polygon([(-2, -2), (-2, -1), (-1, -1), (-1, -2)])
    r2 = PolygonalRegion(polygon=p2)

    r3 = BoxRegion(dimensions=(5, 3, 5))

    r = r3.intersect(r1.union(r2))

    assert isinstance(r, PolygonalRegion)

    for _ in range(100):
        point = r.uniformPointInner()
        assert r.containsPoint(point)
        assert r1.containsPoint(point) or r2.containsPoint(point)
        assert r3.containsPoint(point)


def test_mesh_polygonal_footprint_intersection():
    r1 = BoxRegion(position=(0, 0, 0), dimensions=(3, 3, 2))
    r2 = CircularRegion((0, 0), 1, resolution=64).footprint

    r = r1.intersect(r2)

    assert isinstance(r, MeshVolumeRegion)

    v_pts = list(trimesh.sample.volume_mesh(r.mesh, 100))
    s_pts = [r.getSurfaceRegion().uniformPointInner() for _ in range(100)]

    for x, y, z in v_pts:
        assert math.hypot(x, y) <= 1
        assert -1 <= z <= 1
        assert r1.containsPoint((x, y, z))
        assert r2.containsPoint((x, y, z))

    for x, y, z in s_pts:
        on_side = math.hypot(x, y) == pytest.approx(1, abs=1e-4)
        on_top_bottom = z == -1 or z == 1

        assert on_side or on_top_bottom


def test_mesh_polygonal_footprints_intersection():
    p1 = shapely.geometry.Polygon([(1, 1), (1, 2), (2, 2), (2, 1)])
    r1 = PolygonalRegion(polygon=p1).footprint

    p2 = shapely.geometry.Polygon([(-2, -2), (-2, -1), (-1, -1), (-1, -2)])
    r2 = PolygonalRegion(polygon=p2).footprint

    r3 = BoxRegion(dimensions=(5, 3, 5))

    r = r3.intersect(r1.union(r2))

    assert isinstance(r, MeshVolumeRegion)

    for point in list(trimesh.sample.volume_mesh(r.mesh, 100)):
        assert r.containsPoint(point)
        assert r1.containsPoint(point) or r2.containsPoint(point)
        assert r3.containsPoint(point)


def test_polygon_polygonal_footprint_intersection():
    r1 = CircularRegion((0, 0, 0), 1, resolution=64)
    r2 = RectangularRegion((0, 0, 0), 0, 10, 10)

    assert r1.intersect(r2.footprint).polygons.equals(r1.polygons)
    assert r1.footprint.intersect(r2).polygons.equals(r1.polygons)


def test_polygons_intersection_elevation():
    r1 = CircularRegion((0, 0, 0), 1, resolution=64)
    r2 = CircularRegion((0, 0, 1), 1, resolution=64)

    assert isinstance(r1.intersect(r2), EmptyRegion)


def test_mesh_polyline_intersection():
    point_lists = []

    for y in range(-5, 6, 2):
        point_lists.append([])

        for x in range(-5, 6, 2):
            target_list = point_lists[-1]

            target_list.append(numpy.array((x, y, 0)))
            target_list.append(numpy.array((x, y + 1, 0)))
            target_list.append(numpy.array((x + 1, y + 1, 0)))
            target_list.append(numpy.array((x + 1, y, 0)))

    r1 = PolylineRegion(polyline=shapely.ops.linemerge(point_lists))
    r2 = SpheroidRegion(dimensions=(5, 5, 5))

    r = r1.intersect(r2)

    assert isinstance(r, PolylineRegion)

    for _ in range(100):
        point = r.uniformPointInner()
        assert r.containsPoint(point)
        assert r1.containsPoint(point)
        assert r2.containsPoint(point)


def test_mesh_path_intersection():
    polyline_list = []

    for z in range(-2, 3):
        polyline_list.append([])
        target_list = polyline_list[-1]
        for y in range(-5, 6, 2):
            for x in range(-5, 6, 2):
                target_list.append((x, y, 0))
                target_list.append((x, y + 1, 0))
                target_list.append((x + 1, y + 1, 0))
                target_list.append((x + 1, y, 0))

    r1 = PathRegion(polylines=polyline_list)
    r2 = SpheroidRegion(dimensions=(5, 5, 5))

    r = r1.intersect(r2)

    assert isinstance(r, PathRegion)

    for _ in range(100):
        point = r.uniformPointInner()
        assert r.containsPoint(point)
        assert r1.containsPoint(point)
        assert r2.containsPoint(point)


def test_view_region_construction():
    sampleSceneFrom(
        """
        workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
        workspace = Workspace(workspace_region)

        sample_space = BoxRegion(dimensions=(30,30,30), position=(0,0,15))

        ego = new Object with visibleDistance 20,
            with width 5,
            with length 5,
            with height 5,
            with viewAngles (360 deg, 180 deg)

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (360 deg, 90 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with viewAngles (180 deg, 180 deg),
            with requireVisible True,
            with cameraOffset (0,0,0.5)

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (180 deg, 90 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (200 deg, 180 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (20 deg, 180 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (90 deg, 90 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (200 deg, 40 deg),
            with requireVisible True
    """,
        maxIterations=1000,
    )


def test_pointset_region():
    ps = PointSetRegion("ps", [(1, 2), (3, 4), (5, 6)])
    assert ps in {ps}
    for pt in [(1, 2), (3, 4), (5, 6), (1 + 1e-8, 2 + 1e-8)]:
        assert ps.containsPoint(pt)
    for pt in [(2, 3), (1, 2.01), (0, 0)]:
        assert not ps.containsPoint(pt)
    assert not ps.containsObject(Object._with(position=(1, 2)))
    assert ps.distanceTo((3, 4)) == 0
    assert ps.distanceTo((3, 5)) == pytest.approx(1)
    assert ps.distanceTo((2, 3)) == pytest.approx(math.sqrt(2))


# General properties of regions


def test_orientation_inheritance():
    v = VectorField("foo", lambda pos: pos.x)
    r = RectangularRegion((0, 0), 0, 2, 2)
    r.orientation = v
    c = CircularRegion((1, 0), 1)
    i = r.intersect(c)
    assert i.orientation is v
    v2 = VectorField("bar", lambda pos: pos.y)
    c.orientation = v2
    assert r.intersect(c).orientation is v
    assert c.intersect(r).orientation is v2
