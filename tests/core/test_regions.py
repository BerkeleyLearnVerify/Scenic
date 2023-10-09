import math
from pathlib import Path

import pytest
import shapely.geometry

from scenic.core.object_types import Object, OrientedPoint
from scenic.core.regions import *
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
    assert circ.AABB == ((2, -5), (6, -1), (0, 0))


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
    assert CircularRegion(Vector(4, 5, 2), 2).uniformPointInner().z == 2


def test_sector_sampling():
    assert SectorRegion(Vector(4, 5, 2), 2, 1, 1).uniformPointInner().z == 2


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
    (minx, miny), (maxx, maxy), _ = rect.AABB
    assert maxy == pytest.approx(3 + math.sqrt(3) / 2)
    assert miny == pytest.approx(1 - math.sqrt(3) / 2)
    assert maxx == pytest.approx(1.5 + math.sqrt(3))
    assert minx == pytest.approx(0.5 - math.sqrt(3))
    assert RectangularRegion(Vector(2.5, 4.5, 3), 1, 1, 1).uniformPointInner().z == 3


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
    assert pl.AABB == ((0, 0), (1, 2), (0, 0))
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
    assert poly.distanceTo((0, 0)) == pytest.approx(math.sqrt(2))
    assert poly.distanceTo((0, 0, -1)) == pytest.approx(math.sqrt(3))
    assert poly.distanceTo((2, 1.1, 4)) == pytest.approx(4)
    assert poly.containsObject(Object._with(position=(2, 1.25), width=0.49, length=0.49))
    assert not poly.containsObject(Object._with(position=(2, 1.25), width=1, length=0.49))
    assert poly.AABB == ((1, 1), (3, 2), (0, 0))
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
    assert (
        PolygonalRegion([(1, 1), (3, 1), (2, 2), (1.3, 1.15)], z=3).uniformPointInner().z
        == 3
    )


def test_polygon_unionAll():
    poly1 = PolygonalRegion([(1, 0), (1, 1), (2, 1), (2, 0)], z=2)
    poly2 = PolygonalRegion([(-1, 0), (-1, 1), (0, 1), (0, 0)], z=2)
    union = PolygonalRegion.unionAll((poly1, nowhere, poly2))
    assert isinstance(union, PolygonalRegion)
    assert union.z == 2
    assert union.containsPoint((1.5, 0.5))
    assert union.containsPoint((-0.5, 0.5))
    assert not union.containsPoint((0.5, 0.5))

    poly3 = PolygonalRegion([(0, 0), (1, 1), (1, 0)], z=1)
    with pytest.raises(ValueError):
        PolygonalRegion.unionAll((poly1, poly3))

    with pytest.raises(TypeError):
        PolygonalRegion.unionAll((poly1, everywhere))


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


def test_mesh_region_fromFile(getAssetPath):
    MeshVolumeRegion.fromFile(
        getAssetPath("meshes/classic_plane.obj.bz2"),
        dimensions=(20, 20, 10),
        rotation=(math.radians(-90), 0, math.radians(-10)),
    )
    MeshSurfaceRegion.fromFile(
        getAssetPath("meshes/classic_plane.obj.bz2"),
        dimensions=(20, 20, 10),
        rotation=(math.radians(-90), 0, math.radians(-10)),
    )


def test_mesh_volume_region_zero_dimension():
    for dims in ((0, 1, 1), (1, 0, 1), (1, 1, 0)):
        with pytest.raises(ValueError):
            BoxRegion(dimensions=dims)


def test_mesh_surface_region_negative_dimension():
    mesh = trimesh.creation.box((0.5, 0.5, 0.5))
    for dims in ((-1, 1, 1), (1, -1, 1), (1, 1, -1)):
        with pytest.raises(ValueError):
            MeshSurfaceRegion(mesh, dimensions=dims)


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


def test_mesh_intersects():
    r1 = BoxRegion(dimensions=(1, 1, 1))
    r2 = BoxRegion(dimensions=(2, 2, 2))

    assert r1.intersects(r2)
    assert r1.getSurfaceRegion().intersects(r2)
    assert not r1.intersects(r2.getSurfaceRegion())
    assert not r1.getSurfaceRegion().intersects(r2.getSurfaceRegion())


def test_mesh_empty_intersection():
    for engine in ["blender", "scad"]:
        r1 = BoxRegion(position=(0, 0, 0), engine=engine)
        r2 = BoxRegion(position=(10, 10, 10), engine=engine)

        assert isinstance(r1.intersect(r2), EmptyRegion)


def test_mesh_empty_difference():
    for engine in ["blender", "scad"]:
        r1 = BoxRegion(dimensions=(1, 1, 1), engine=engine)
        r2 = BoxRegion(dimensions=(2, 2, 2), engine=engine)

        assert isinstance(r1.difference(r2), EmptyRegion)


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

    with pytest.raises(TypeError):
        PathRegion(points=polylines)

    with pytest.raises(TypeError):
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

    # Test distanceTo
    assert r2.distanceTo(Vector(3.5, 3.5, 1.5)) == pytest.approx(0.5)
    assert r2.distanceTo(Vector(4, 4, 2)) == pytest.approx(0)
    assert r2.distanceTo(Vector(5, 5, 2)) == pytest.approx(math.sqrt(2))
    assert r2.distanceTo(Vector(5, 5, 3)) == pytest.approx(math.sqrt(3))
    assert r2.distanceTo(Vector(3, 0, 0)) == pytest.approx(3)
    assert r2.distanceTo(Vector(0, 3, 0)) == pytest.approx(3)
    assert r2.distanceTo(Vector(0, 0, 0)) == pytest.approx(math.sqrt(18))

    # Test AABB
    assert r1.AABB == ((0, 1), (0, 1), (0, 0))
    assert r2.AABB == ((3, 4), (3, 4), (0, 3))
    assert r3.AABB == ((6, 7), (6, 7), (0, 3))
    assert r4.AABB == ((0, 7), (0, 7), (0, 3))


def test_mesh_polygon_intersection():
    r1 = BoxRegion(position=(0, 0, 0), dimensions=(3, 0.5, 2))
    r2 = CircularRegion((0, 0), 1, resolution=64)

    r = r1.intersect(r2)

    assert isinstance(r, PolygonalRegion)

    v_pts = [r.uniformPointInner() for _ in range(100)]

    for x, y, z in v_pts:
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


def test_pointset_region():
    PointSetRegion("foo", [(1, 2), (3, 4), (5, 6)])
    PointSetRegion("bar", [(1, 2, 1), (3, 4, 2), (5, 6, 3)])
    ps = PointSetRegion("ps", [(1, 2), (3, 4), (5, 6), (1, 5, 5)])
    assert ps in {ps}
    for pt in [(1, 2), (3, 4), (5, 6), (1 + 1e-8, 2 + 1e-8), (1, 5, 5)]:
        assert ps.containsPoint(pt)
    for pt in [(2, 3), (1, 2.01), (0, 0), (1, 5)]:
        assert not ps.containsPoint(pt)
    assert not ps.containsObject(Object._with(position=(1, 2)))
    assert ps.distanceTo((3, 4)) == 0
    assert ps.distanceTo((3, 5)) == pytest.approx(1)
    assert ps.distanceTo((2, 3)) == pytest.approx(math.sqrt(2))
    assert ps.AABB == ((1, 5), (2, 6), (0, 5))


# ViewRegion tests
H_ANGLES = [0.1, 45, 90, 135, 179.9, 180, 180.1, 225, 270, 315, 359.9, 360]

V_ANGLES = [0.1, 45, 90, 135, 179.9, 180]

VISIBLE_DISTANCES = [1, 25, 50]


@pytest.mark.exhaustive
@pytest.mark.parametrize(
    "hAngle,vAngle,visibleDistance",
    itertools.product(H_ANGLES, V_ANGLES, VISIBLE_DISTANCES),
)
def test_viewRegion_full(hAngle, vAngle, visibleDistance):
    viewRegion_test_helper(hAngle, vAngle, visibleDistance)


def test_viewRegion_minimal():
    viewRegion_test_helper(90, 90, 50)


def viewRegion_test_helper(hAngle, vAngle, visibleDistance):
    hAngle = math.radians(hAngle)
    vAngle = math.radians(vAngle)

    sphere = SpheroidRegion(dimensions=(100, 100, 100))
    vr = ViewRegion(visibleDistance, (hAngle, vAngle))
    vr_surface = vr.getSurfaceRegion()

    for pt in trimesh.sample.volume_mesh(sphere.mesh, 1000):
        x, y, z = pt
        azimuth = -math.atan2(x, y)
        altitude = math.atan2(z, math.hypot(x, y))
        distance = math.hypot(*pt)

        pt_contained = (
            (-hAngle / 2) <= azimuth <= (hAngle / 2)
            and (-vAngle / 2) <= altitude <= (vAngle / 2)
            and distance <= visibleDistance
        )

        if vr_surface.distanceTo(pt) > 0.01 * visibleDistance:
            assert vr.containsPoint(pt) == pt_contained


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


# General test of region combinations

REGIONS = {
    MeshVolumeRegion: MeshVolumeRegion(trimesh.creation.box((0.75, 0.75, 0.75))),
    MeshSurfaceRegion: MeshSurfaceRegion(trimesh.creation.box((0.5, 0.5, 0.5))),
    BoxRegion: BoxRegion(),
    SpheroidRegion: SpheroidRegion(),
    PolygonalFootprintRegion: PolygonalRegion(
        [(0, 0.5), (0, 1), (2, 1), (0, 0)]
    ).footprint,
    PathRegion: PathRegion(points=[(6, 6), (6, 7, 1), (7, 7, 2), [7, 6, 3]]),
    PolygonalRegion: PolygonalRegion([(0, 0.5), (0, 1), (2, 1), (0, 0)]),
    CircularRegion: CircularRegion(Vector(29, 34), 5),
    SectorRegion: SectorRegion(Vector(29, 34), 5, 1, 0.5),
    RectangularRegion: RectangularRegion(Vector(1, 2), math.radians(30), 4, 2),
    PolylineRegion: PolylineRegion([(0, 2), (1, 1), (0, 0)]),
    PointSetRegion: PointSetRegion("foo", [(1, 2), (3, 4), (5, 6)]),
    ViewRegion: ViewRegion(50, (1, 1)),
}

INVALID_INTERSECTS = (
    {MeshSurfaceRegion, PathRegion},
    {MeshSurfaceRegion, PolygonalRegion},
    {MeshSurfaceRegion, CircularRegion},
    {MeshSurfaceRegion, SectorRegion},
    {MeshSurfaceRegion, RectangularRegion},
    {MeshSurfaceRegion, PolylineRegion},
    {PathRegion, PolygonalRegion},
    {PathRegion, CircularRegion},
    {PathRegion, SectorRegion},
    {PathRegion, RectangularRegion},
    {PathRegion, PolylineRegion},
)


def regions_id(val):
    return val[0].__name__


@pytest.mark.slow
@pytest.mark.parametrize(
    "A,B", itertools.combinations(REGIONS.items(), 2), ids=regions_id
)
def test_region_combinations(A, B):
    type_a, region_a = A
    type_b, region_b = B

    ## Check type correctness ##
    assert isinstance(region_a, type_a)
    assert isinstance(region_b, type_b)

    ## Check all output combinations ##
    # intersects()
    try:
        intersects_out_1 = region_a.intersects(region_b)
        intersects_out_2 = region_b.intersects(region_a)
        assert intersects_out_1 == intersects_out_2
        assert isinstance(intersects_out_1, bool)
    except NotImplementedError:
        assert set([type_a, type_b]) in INVALID_INTERSECTS

    # intersection()
    intersect_out_1 = region_a.intersect(region_b)
    intersect_out_2 = region_b.intersect(region_a)
    assert type(intersect_out_1) == type(intersect_out_2)
    assert isinstance(intersect_out_1, Region)

    # union()
    union_out_1 = region_a.union(region_b)
    union_out_2 = region_b.union(region_a)
    assert type(union_out_1) == type(union_out_2)
    assert isinstance(union_out_1, Region)

    # difference()
    difference_out = region_a.difference(region_b)
    assert isinstance(difference_out, Region)
