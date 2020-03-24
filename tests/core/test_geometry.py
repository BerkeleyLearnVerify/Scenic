
import pytest
import shapely.geometry
import shapely.ops

import scenic.core.geometry as geometry

def checkTriangulation(poly):
    tris = geometry.triangulatePolygon(poly)
    assert all(isinstance(t, shapely.geometry.Polygon) for t in tris)
    assert all(len(t.exterior.coords) == 4 for t in tris)
    assert all(len(t.interiors) == 0 for t in tris)
    assert all(poly.contains(t) for t in tris)
    for i, t1 in enumerate(tris[:-1]):
        t2 = tris[i+1]
        assert (t1 & t2).area == pytest.approx(0)
    union = shapely.ops.unary_union(tris)
    assert poly.difference(union).area == pytest.approx(0)
    assert union.difference(poly).area == pytest.approx(0)
    return tris

def test_triangulation():
    p = shapely.geometry.Polygon([(0,0), (0,1), (1,1), (1,0)])
    tris = checkTriangulation(p)
    assert len(tris) == 2

def test_triangulation_hole():
    p = shapely.geometry.Polygon(
        [(0,0), (0,3), (3,3), (3,0)],
        holes=[[(1,1), (1,2), (2,2), (2,1)]]
    )
    checkTriangulation(p)
