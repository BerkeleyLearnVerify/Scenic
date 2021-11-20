
import math

import pytest

import scenic
from scenic.core.errors import InvalidScenarioError
from tests.utils import compileScenic, sampleScene, sampleEgoFrom

# Built-in regions

def test_everywhere():
    scenario = compileScenic('ego = Object with regionContainedIn everywhere')
    sampleScene(scenario, maxIterations=1)

def test_nowhere():
    with pytest.raises(InvalidScenarioError):
        compileScenic('ego = Object with regionContainedIn nowhere')

# PolygonalRegion

def test_polygonal_empty_intersection():
    scenario = compileScenic("""
        r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])
        ego = Object at -10@0, facing Range(-90, 0) deg, with viewAngle 60 deg
        Object in visible r1, with requireVisible False
    """)
    for i in range(30):
        sampleScene(scenario, maxIterations=1000)

# PolylineRegion

def test_polyline_start():
    ego = sampleEgoFrom("""
        r = PolylineRegion([1@1, 3@-1, 6@2])
        pt = r.start
        ego = Object at pt, facing pt.heading
    """)
    assert tuple(ego.position) == pytest.approx((1, 1))
    assert ego.heading == pytest.approx(math.radians(-135))

def test_polyline_end():
    ego = sampleEgoFrom("""
        r = PolylineRegion([1@1, 3@-1, 6@2])
        pt = r.end
        ego = Object at pt, facing pt.heading
    """)
    assert tuple(ego.position) == pytest.approx((6, 2))
    assert ego.heading == pytest.approx(math.radians(-45))
