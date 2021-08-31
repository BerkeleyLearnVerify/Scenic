
import pytest

import scenic
from scenic.core.errors import ScenicSyntaxError, InvalidScenarioError
from tests.utils import compileScenic, sampleScene, sampleSceneFrom, sampleEgo

## Basic

def test_requirement():
    scenario = compileScenic("""
        ego = Object at Range(-10, 10) @ 0
        require ego.position.x >= 0
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(0 <= x <= 10 for x in xs)

@pytest.mark.slow
def test_soft_requirement():
    scenario = compileScenic("""
        ego = Object at Range(-10, 10) @ 0
        require[0.9] ego.position.x >= 0
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(350)]
    count = sum(x >= 0 for x in xs)
    assert 255 <= count < 350

def test_named_requirement():
    scenario = compileScenic("""
        ego = Object at Range(0, 10) @ 0
        require ego.position.x >= 5 as posReq
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(5 <= x <= 10 for x in xs)

@pytest.mark.slow
def test_named_soft_requirement():
    scenario = compileScenic("""
        ego = Object at Range(0, 10) @ 0
        require[0.9] ego.position.x >= 5 as posReq
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(350)]
    count = sum(x >= 5 for x in xs)
    assert 255 <= count < 350

## Forbidden operations inside requirements

def test_object_in_requirement():
    scenario = compileScenic("""
        require Object
        ego = Object
    """)
    with pytest.raises(ScenicSyntaxError):
        sampleScene(scenario)

def test_param_in_requirement():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("""
            require param x = 4
            ego = Object
        """)

@pytest.mark.xfail(reason='looser keyword policy now allows this', strict=True)
def test_mutate_in_requirement():
    scenario = compileScenic("""
        require mutate
        ego = Object
    """)
    with pytest.raises(ScenicSyntaxError):
        sampleScene(scenario)

def test_require_in_requirement():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("""
            require (require True)
            ego = Object
        """)

## Error handling

def test_runtime_parse_error_in_requirement():
    scenario = compileScenic("""
        require visible 4
        ego = Object
    """)
    with pytest.raises(ScenicSyntaxError):
        sampleScene(scenario)

## Enforcement of built-in requirements

def test_containment_requirement():
    scenario = compileScenic("""
        foo = RectangularRegion(0@0, 0, 10, 10)
        ego = Object at Range(0, 10) @ 0, with regionContainedIn foo
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(0 <= x <= 5 for x in xs)

def test_visibility_requirement():
    scenario = compileScenic("""
        ego = Object with visibleDistance 10, with viewAngle 90 deg, facing 45 deg
        other = Object at Range(-10, 10) @ 0
    """)
    xs = [sampleScene(scenario, maxIterations=60).objects[1].position.x for i in range(60)]
    assert all(-10 <= x <= 0.5 for x in xs)

def test_visibility_requirement_disabled():
    scenario = compileScenic("""
        ego = Object with visibleDistance 10, with viewAngle 90 deg, facing 45 deg
        other = Object at Range(-10, 10) @ 0, with requireVisible False
    """)
    xs = [sampleScene(scenario, maxIterations=60).objects[1].position.x for i in range(60)]
    assert any(x > 0.5 for x in xs)

def test_intersection_requirement():
    scenario = compileScenic("""
        ego = Object at Range(0, 2) @ 0
        other = Object
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert all(x >= 1 for x in xs)

def test_intersection_requirement_disabled_1():
    scenario = compileScenic("""
        ego = Object at Range(0, 2) @ 0, with allowCollisions True
        other = Object
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert any(x < 1 for x in xs)

def test_intersection_requirement_disabled_2():
    scenario = compileScenic("""
        ego = Object at Range(0, 2) @ 0
        other = Object with allowCollisions True
    """)
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(60)]
    assert any(x < 1 for x in xs)

## Static violations of built-in requirements

def test_static_containment_violation():
    with pytest.raises(InvalidScenarioError):
        compileScenic("""
            foo = RectangularRegion(0@0, 0, 5, 5)
            ego = Object at 10@10, with regionContainedIn foo
        """)

def test_static_empty_container():
    with pytest.raises(InvalidScenarioError):
        compileScenic("""
            foo = PolylineRegion([0@0, 1@1]).intersect(PolylineRegion([1@0, 2@1]))
            ego = Object at Range(0, 2) @ Range(0, 1), with regionContainedIn foo
        """)

def test_static_visibility_violation():
    with pytest.raises(InvalidScenarioError):
        compileScenic("""
            ego = Object at 10@0, facing -90 deg, with viewAngle 90 deg
            Object at 0@10
        """)

def test_static_visibility_violation_disabled():
    sampleSceneFrom("""
        ego = Object at 10@0, facing -90 deg, with viewAngle 90 deg
        Object at 0@10, with requireVisible False
    """)

def test_static_intersection_violation():
    with pytest.raises(InvalidScenarioError):
        compileScenic("""
            ego = Object at 0@0
            Object at 1@0
        """)

def test_static_intersection_violation_disabled():
    sampleSceneFrom("""
        ego = Object at 0@0
        Object at 1@0, with allowCollisions True
    """)
