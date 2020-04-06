
import pytest
import shapely.geometry
import shapely.ops

import scenic.core.geometry as geometry

def checkTriangulation(poly, prec=1e-6):
    tris = geometry.triangulatePolygon(poly)
    assert all(isinstance(t, shapely.geometry.Polygon) for t in tris)
    assert all(len(t.exterior.coords) == 4 for t in tris)
    assert all(len(t.interiors) == 0 for t in tris)
    # check triangles are (nearly) contained in poly
    assert all(t.difference(poly).area == pytest.approx(0, abs=prec) for t in tris)
    # check triangles are (nearly) disjoint
    for i, t1 in enumerate(tris[:-1]):
        for t2 in tris[i+1:]:
            assert (t1 & t2).area == pytest.approx(0, abs=prec)
    # check union of triangles is (nearly) identical to poly
    union = shapely.ops.unary_union(tris)
    assert poly.difference(union).area == pytest.approx(0, abs=prec)
    assert union.difference(poly).area == pytest.approx(0, abs=prec)
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

def test_triangulation_hole_2():
    """An example where naive Delaunay point set triangulation fails."""
    p = shapely.geometry.Polygon(
        [(-1,0), (0,3), (1,0), (0,-3)],
        holes=[[(0,2),(0.2,-2),(-0.2,-2)]]
    )
    checkTriangulation(p)

def test_triangulation_holes():
    """An example with multiple holes."""
    p = shapely.geometry.Polygon(
        [(0,0), (0,3), (5,3), (5,0)],
        holes=[
            [(1,1), (1,2), (2,2), (2,1)],
            [(3,1), (3,2), (4,2), (4,1)],
        ]
    )
    checkTriangulation(p)
