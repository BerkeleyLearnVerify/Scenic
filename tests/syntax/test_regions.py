
import math

import pytest

import scenic
from scenic.core.errors import InvalidScenarioError
from scenic.core.vectors import Vector
from tests.utils import compileScenic, sampleScene, sampleEgo, sampleEgoFrom

# Built-in regions

def test_everywhere():
    scenario = compileScenic('ego = new Object with regionContainedIn everywhere')
    sampleScene(scenario, maxIterations=1)

def test_nowhere():
    with pytest.raises(InvalidScenarioError):
        compileScenic('ego = new Object with regionContainedIn nowhere')

# CircularRegion

def test_circular_in():
    scenario = compileScenic('ego = new Object in CircularRegion(3@5, 2)')
    positions = [sampleEgo(scenario).position for i in range(30)]
    center = Vector(3, 5)
    distances = [pos.distanceTo(center) for pos in positions]
    assert all(dist <= 2 for dist in distances)
    assert any(dist < 1.5 for dist in distances)
    assert any(dist > 1.5 for dist in distances)
    assert any(pos.x < 3 for pos in positions)
    assert any(pos.x > 3 for pos in positions)

def test_circular_lazy():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 2 * pos.x)
        x = 0 relative to vf
        ego = new Object at Range(-1, 1) @ 0, with foo CircularRegion(0@0, x)
    """)
    assert ego.foo.radius == 2 * ego.position.x

# PolygonalRegion

def test_polygonal_empty_intersection():
    scenario = compileScenic("""
        r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])
        ego = new Object at -10@0, facing Range(-90, 0) deg, with viewAngle 60 deg
        new Object in visible r1, with requireVisible False
    """)
    for i in range(30):
        sampleScene(scenario, maxIterations=1000)

# PolylineRegion

def test_polyline_start():
    ego = sampleEgoFrom("""
        r = PolylineRegion([1@1, 3@-1, 6@2])
        pt = r.start
        ego = new Object at pt, facing pt.heading
    """)
    assert tuple(ego.position) == pytest.approx((1, 1))
    assert ego.heading == pytest.approx(math.radians(-135))

def test_polyline_end():
    ego = sampleEgoFrom("""
        r = PolylineRegion([1@1, 3@-1, 6@2])
        pt = r.end
        ego = new Object at pt, facing pt.heading
    """)
    assert tuple(ego.position) == pytest.approx((6, 2))
    assert ego.heading == pytest.approx(math.radians(-45))
