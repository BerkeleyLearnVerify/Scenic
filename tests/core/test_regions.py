
import shapely.geometry

from scenic.core.regions import *

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
