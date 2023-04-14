
import math

import pytest
import shapely.geometry

from scenic.core.object_types import Object, OrientedPoint
from scenic.core.regions import *
from scenic.core.vectors import VectorField

# Particular region types

def test_all_region():
    ar = AllRegion('all')
    assert ar in {ar}   # check hashability
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
    assert ar.containsObject(Object())
    assert circ.difference(ar) == nowhere
    assert ar.union(circ) is ar
    assert circ.union(ar) is ar
    assert ar.distanceTo(Vector(4, 12)) == 0

def test_empty_region():
    er = EmptyRegion('none')
    assert er in {er}
    assert er == nowhere
    assert not er.containsPoint(Vector(0, 0))
    assert not er.containsObject(Object())
    circ = CircularRegion(Vector(29, 34), 5)
    assert er.intersect(circ) is er
    assert circ.intersect(er) is er
    assert not er.intersects(circ)
    assert not circ.intersects(er)
    assert circ.difference(er) is circ
    assert er.difference(circ) is er
    assert er.union(circ) is circ
    assert circ.union(er) is circ
    assert er.distanceTo(Vector(4, 12)) == float('inf')

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
    assert circ.getAABB() == ((2, -5), (6, -1))

def test_circular_sampling():
    center = Vector(4, -3)
    circ = CircularRegion(center, 2)
    pts = [circ.uniformPointInner() for i in range(3000)]
    dists = [pt.distanceTo(center) for pt in pts]
    assert all(dist <= 2 for dist in dists)
    assert sum(dist <= 1.414 for dist in dists) >= 1250
    assert sum(dist > 1.414 for dist in dists) >= 1250
    xs, ys = zip(*pts)
    assert sum(x >= 4 for x in xs) >= 1250
    assert sum(y >= -3 for y in ys) >= 1250

def test_rectangular_region():
    rect = RectangularRegion(Vector(1, 2), math.radians(30), 4, 2)
    assert rect in {rect}
    for pt in [(1,2), (2.2,3.8), (3.1,2.15), (-1.1,1.85), (1,3.1)]:
        assert rect.containsPoint(pt)
    for pt in [(1,3.5), (2.8,3), (-0.5,2.3)]:
        assert not rect.containsPoint(pt)
    r2 = RectangularRegion(Vector(3.5, 2.5), 0, 1, 1)
    assert rect.intersects(r2)
    r3 = RectangularRegion(Vector(2.5, 4.5), 0, 1, 1)
    assert not rect.intersects(r3)
    assert rect.distanceTo((1 + 2*math.sqrt(3), 4)) == pytest.approx(2)
    ((minx, miny), (maxx, maxy)) = rect.getAABB()
    assert maxy == pytest.approx(3 + math.sqrt(3)/2)
    assert miny == pytest.approx(1 - math.sqrt(3)/2)
    assert maxx == pytest.approx(1.5 + math.sqrt(3))
    assert minx == pytest.approx(0.5 - math.sqrt(3))

def test_polyline_region():
    # With standard orientation
    pl = PolylineRegion([(0,2), (1,1), (0,0)])
    assert pl in {pl}
    assert len(pl) == 3
    for pt in ((0,2), (0.5,1.5), (1,1), (0.5,0.5), (0,0)):
        assert pl.containsPoint(pt)
    for pt in ((1,2), (1,0), (0,1), (0.5,1), (2,2), (-1,-1)):
        assert not pl.containsPoint(pt)
    assert not pl.containsObject(Object())
    assert pl.project((0, 3)) == (0, 2)
    assert pl.distanceTo((0, 3)) == pytest.approx(1)
    assert pl.project((0.3, 0.2)) == pytest.approx((0.25, 0.25))
    assert pl.distanceTo((0.3, 0.2)) == pytest.approx(0.05 * math.sqrt(2))
    assert pl.pointAlongBy(math.sqrt(2)) == pytest.approx((1, 1))
    assert pl.pointAlongBy(0.5, normalized=True) == pytest.approx((1, 1))
    assert pl.pointAlongBy(0.75, normalized=True) == pytest.approx((0.5, 0.5))
    assert pl.equallySpacedPoints(3) == list(pl.points)
    assert pl.pointsSeparatedBy(math.sqrt(2)) == list(pl.points[:-1])
    assert pl.length == pytest.approx(2*math.sqrt(2))
    assert pl.getAABB() == ((0, 0), (1, 2))
    start = pl.start
    assert isinstance(start, OrientedPoint)
    assert start.position == (0, 2)
    assert start.heading == pytest.approx(math.radians(-135))
    end = pl.end
    assert isinstance(end, OrientedPoint)
    assert end.position == (0, 0)
    assert end.heading == pytest.approx(math.radians(135))
    # With non-standard orientation
    vf = VectorField('foo', lambda pos: 0.25 + (pos.x + pos.y) / 2)
    pl2 = PolylineRegion([(1,1), (1,-1), (-1,-1), (-1,1)], orientation=vf)
    assert not pl2.containsObject(Object(width=2, length=2))
    assert pl2.start.heading == pytest.approx(1.25)
    assert pl2.end.heading == pytest.approx(0.25)
    assert pl.intersects(pl2)
    assert pl2.intersects(pl)
    # With no orientation
    pl = PolylineRegion([(0,0), (-1,1), (1,1)], orientation=None)
    assert pl.start.heading == 0
    assert pl.end.heading == 0
    poly = PolygonalRegion([(0,0.5), (0,1), (2,1), (0,0)])
    assert pl.intersects(poly)
    assert poly.intersects(pl)
    i = pl.intersect(poly)
    assert isinstance(i, PolylineRegion)
    assert set(i.points) == {(0,1), (1,1)}
    d = pl.difference(poly)
    assert isinstance(d, PolylineRegion)
    assert set(d.points) == {(0,0), (-1,1), (0,1)}
    assert PolylineRegion.unionAll(()) == nowhere
    pl2 = PolylineRegion([(2,0), (3,0)])
    union = PolylineRegion.unionAll([pl, pl2])
    assert isinstance(union, PolylineRegion)
    assert union.points[:3] == pl.points
    assert union.points[3:] == pl2.points
    assert pl + pl2 == union
    pl3 = PolylineRegion([(3,0), (3,1)])
    u2 = PolylineRegion.unionAll([union, pl3])
    assert isinstance(u2, PolylineRegion)
    assert u2.points[:5] == union.points
    assert u2.points[4:] == pl3.points
    assert union + pl3 == u2
    assert pl2.intersects(pl3)
    assert pl3.intersects(pl2)
    i = pl2.intersect(pl3)
    assert isinstance(i, PointSetRegion)
    assert i.points.tolist() == [[3,0]]
    pl4 = PolylineRegion([(3,1), (3,0)])
    assert pl3 != pl4
    assert pl3.difference(pl4) == nowhere
    pl5 = PolylineRegion([(2,1), (4,0.5), (2,0)])
    i = pl4.intersect(pl5)
    assert isinstance(i, PointSetRegion)
    assert len(i.points) == 2

def test_polygon_region():
    poly = PolygonalRegion([(1,1), (3,1), (2,2), (1.3,1.15)])
    assert poly in {poly}
    for pt in [(1,1), (2,1), (2,1.5), (2.5,1.4)]:
        assert poly.containsPoint(pt)
    for pt in [(1,1.1), (2,0.9), (2,2.1), (2.9, 1.2), (1.5,1.4)]:
        assert not poly.containsPoint(pt)
    assert poly.containsObject(Object(position=(2,1.25), width=0.5, length=0.5))
    assert not poly.containsObject(Object(position=(2,1.25), width=1, length=0.5))
    assert poly.getAABB() == ((1, 1), (3, 2))
    line = PolylineRegion([(1,1), (2,1.8)])
    assert poly.intersects(line)
    assert line.intersects(poly)
    i = poly.intersect(line)
    assert isinstance(i, PolylineRegion)
    poly2 = PolygonalRegion([(1,1), (2,1.8), (3,2), (1,2)])
    assert poly.intersects(poly2)
    i = poly.intersect(poly2)
    assert isinstance(i, PolygonalRegion)
    assert i.containsPoint((2,2))
    d = poly.difference(poly2)
    assert isinstance(d, PolygonalRegion)
    assert not d.containsPoint((2,2))

def test_polygon_sampling():
    p = shapely.geometry.Polygon(
        [(0,0), (0,3), (3,3), (3,0)],
        holes=[[(1,1), (1,2), (2,2), (2,1)]]
    )
    r = PolygonalRegion(polygon=p)
    pts = [r.uniformPointInner() for i in range(3000)]
    for x, y in pts:
        assert 0 <= x <= 3 and 0 <= y <= 3
        assert not (1 < x < 2 and 1 < y < 2)
    xs, ys = zip(*pts)
    assert sum(1 <= x <= 2 for x in xs) <= 870
    assert sum(1 <= y <= 2 for y in ys) <= 870
    assert sum(x >= 1.5 for x in xs) >= 1250
    assert sum(y >= 1.5 for y in ys) >= 1250

def test_pointset_region():
    ps = PointSetRegion('ps', [(1,2), (3,4), (5,6)])
    assert ps in {ps}
    for pt in [(1,2), (3,4), (5,6), (1+1e-8,2+1e-8)]:
        assert ps.containsPoint(pt)
    for pt in [(2,3), (1,2.01), (0,0)]:
        assert not ps.containsPoint(pt)
    assert not ps.containsObject(Object(position=(1,2)))
    assert ps.distanceTo((3,4)) == 0
    assert ps.distanceTo((3,5)) == pytest.approx(1)
    assert ps.distanceTo((2,3)) == pytest.approx(math.sqrt(2))

# General properties of regions

def test_orientation_inheritance():
    v = VectorField('foo', lambda pos: pos.x)
    r = RectangularRegion((0, 0), 0, 2, 2)
    r.orientation = v
    c = CircularRegion((1, 0), 1)
    i = r.intersect(c)
    assert i.orientation is v
    v2 = VectorField('bar', lambda pos: pos.y)
    c.orientation = v2
    assert r.intersect(c).orientation is v
    assert c.intersect(r).orientation is v2
