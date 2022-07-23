
import shapely.geometry

from scenic.core.regions import *

def test_all_region():
    ar = AllRegion('all')
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
    assert circ.difference(ar) == nowhere
    assert ar.union(circ) is ar
    assert circ.union(ar) is ar
    assert ar.distanceTo(Vector(4, 12)) == 0

def test_empty_region():
    er = EmptyRegion('none')
    assert er == nowhere
    assert not er.containsPoint(Vector(0, 0))
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
