
import math
import pytest

from scenic.core.errors import RuntimeParseError
from tests.utils import compileScenic, sampleEgoFrom, sampleParamP, sampleParamPFrom

## Scalar operators

# relative heading

def test_relative_heading():
    p = sampleParamPFrom("""
        ego = Object facing 30 deg
        other = Object facing 65 deg, at 10@10
        param p = relative heading of other
    """)
    assert p == pytest.approx(math.radians(65 - 30))

def test_relative_heading_no_ego():
    with pytest.raises(RuntimeParseError):
        compileScenic("""
            other = Object
            ego = Object at 2@2, facing relative heading of other
        """)

def test_relative_heading_from():
    ego = sampleEgoFrom('ego = Object facing relative heading of 70 deg from -10 deg')
    assert ego.heading == pytest.approx(math.radians(70 + 10))

# apparent heading

def test_apparent_heading():
    p = sampleParamPFrom("""
        ego = Object facing 30 deg
        other = Object facing 65 deg, at 10@10
        param p = apparent heading of other
    """)
    assert p == pytest.approx(math.radians(65 + 45))

def test_apparent_heading_no_ego():
    with pytest.raises(RuntimeParseError):
        compileScenic("""
            other = Object
            ego = Object at 2@2, facing apparent heading of other
        """)

def test_apparent_heading_from():
    ego = sampleEgoFrom("""
        OP = OrientedPoint at 10@15, facing -60 deg
        ego = Object facing apparent heading of OP from 15@10
    """)
    assert ego.heading == pytest.approx(math.radians(-60 - 45))

# distance

def test_distance():
    p = sampleParamPFrom("""
        ego = Object at 5@10
        other = Object at 7@-4
        param p = distance to other
    """)
    assert p == pytest.approx(math.hypot(7 - 5, -4 - 10))

def test_distance_no_ego():
    with pytest.raises(RuntimeParseError):
        compileScenic("""
            other = Object
            ego = Object at 2@2, facing distance to other
        """)

def test_distance_from():
    ego = sampleEgoFrom('ego = Object with wobble distance from -3@2 to 4@5')
    assert ego.wobble == pytest.approx(math.hypot(4 - -3, 5 - 2))

def test_distance_to_region():
    p = sampleParamPFrom("""
        r = CircularRegion(10@5, 3)
        ego = Object at 13@9
        param p = distance to r
    """)
    assert p == pytest.approx(2)

def test_distance_past():
    p = sampleParamPFrom("""
        ego = Object at 5@10, facing 45 deg
        other = Object at 2@3
        param p = distance past other
    """)
    assert p == pytest.approx(2 * math.sqrt(2))

def test_distance_past_of():
    p = sampleParamPFrom("""
        ego = Object at 1@2, facing 30 deg
        op = OrientedPoint at 3@-6, facing -45 deg
        param p = distance past ego of op
    """)
    assert p == pytest.approx(-3 * math.sqrt(2))

# angle

def test_angle():
    p = sampleParamPFrom("""
        ego = Object facing 30 deg
        other = Object facing 65 deg, at 10@10
        param p = angle to other
    """)
    assert p == pytest.approx(math.radians(-45))

def test_angle_no_ego():
    with pytest.raises(RuntimeParseError):
        compileScenic("""
            other = Object
            ego = Object at 2@2, facing angle to other
        """)

def test_angle_from():
    ego = sampleEgoFrom('ego = Object facing angle from 2@4 to 3@5')
    assert ego.heading == pytest.approx(math.radians(-45))

## Boolean operators

# can see

def test_point_can_see_vector():
    p = sampleParamPFrom("""
        ego = Object
        pt = Point at 10@20, with visibleDistance 5
        param p = tuple([pt can see 8@19, pt can see 10@26])
    """)
    assert p == (True, False)

def test_point_can_see_object():
    p = sampleParamPFrom("""
        ego = Object with width 10, with length 10
        other = Object at 35@10
        pt = Point at 15@10, with visibleDistance 15
        param p = tuple([pt can see ego, pt can see other])
    """)
    assert p == (True, False)

def test_oriented_point_can_see_vector():
    p = sampleParamPFrom("""
        ego = Object facing -45 deg, with visibleDistance 5, with viewAngle 20 deg
        param p = tuple([ego can see 2@2, ego can see 4@4, ego can see 1@0])
    """)
    assert p == (True, False, False)

def test_oriented_point_can_see_object():
    p = sampleParamPFrom("""
        ego = Object facing -45 deg, with visibleDistance 5, with viewAngle 20 deg
        other = Object at 4@4, with width 2, with length 2
        other2 = Object at 4@0, with requireVisible False
        param p = tuple([ego can see other, ego can see other2])
    """)
    assert p == (True, False)

# in

def test_point_in_region():
    p = sampleParamPFrom("""
        ego = Object
        reg = RectangularRegion(10@5, 0, 4, 2)
        ptA = Point at 11@4.5
        ptB = Point at 11@3.5
        param p = tuple([9@5.5 in reg, 9@7 in reg, ptA in reg, ptB in reg])
    """)
    assert p == (True, False, True, False)

def test_object_in_region():
    p = sampleParamPFrom("""
        reg = RectangularRegion(10@5, 0, 4, 2)
        ego = Object at 11.5@5.5, with width 0.25, with length 0.25
        other = Object at 9@4.5, with width 2.5
        param p = tuple([ego in reg, other in reg])
    """)
    assert p == (True, False)

## Heading operators

# at

def test_field_at_vector():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: (3 * pos.x) + pos.y)
        ego = Object facing (vf at 0.02 @ -1)
    """)
    assert ego.heading == pytest.approx((3 * 0.02) - 1)

# relative to

def test_heading_relative_to_field():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 3 * pos.x)
        ego = Object at 0.07 @ 0, facing 0.5 relative to vf
    """)
    assert ego.heading == pytest.approx(0.5 + (3 * 0.07))

def test_field_relative_to_heading():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 3 * pos.x)
        ego = Object at 0.07 @ 0, facing vf relative to 0.5
    """)
    assert ego.heading == pytest.approx(0.5 + (3 * 0.07))

def test_field_relative_to_field():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 3 * pos.x)
        ego = Object at 0.07 @ 0, facing vf relative to vf
    """)
    assert ego.heading == pytest.approx(2 * (3 * 0.07))

def test_heading_relative_to_heading():
    ego = sampleEgoFrom('ego = Object facing 0.5 relative to -0.3')
    assert ego.heading == pytest.approx(0.5 - 0.3)

def test_heading_relative_to_heading_lazy():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 0.5)
        ego = Object facing 0.5 relative to (0.5 relative to vf)
    """)
    assert ego.heading == pytest.approx(1.5)

def test_mistyped_relative_to():
    with pytest.raises(RuntimeParseError):
        compileScenic('ego = Object facing 0 relative to 1@2')

def test_mistyped_relative_to_lazy():
    with pytest.raises(RuntimeParseError):
        compileScenic("""
            vf = VectorField("Foo", lambda pos: 0.5)
            ego = Object facing 1@2 relative to (0 relative to vf)
        """)

## Vector operators

# offset by

def test_offset_by():
    ego = sampleEgoFrom('ego = Object at 3@2 offset by -4@10')
    assert tuple(ego.position) == pytest.approx((-1, 12))

# offset along

def test_offset_along_heading():
    ego = sampleEgoFrom('ego = Object at 3@2 offset along 45 deg by -4@10')
    d = 1 / math.sqrt(2)
    assert tuple(ego.position) == pytest.approx((3 - 10*d - 4*d, 2 + 10*d - 4*d))

def test_offset_along_field():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 3 deg * pos.x)
        ego = Object at 15@7 offset along vf by 2@-3
    """)
    d = 1 / math.sqrt(2)
    assert tuple(ego.position) == pytest.approx((15 + 3*d + 2*d, 7 - 3*d + 2*d))

# follow

def test_follow():
    ego = sampleEgoFrom("""
        vf = VectorField("Foo", lambda pos: 90 deg * (pos.x + pos.y - 1),
                         minSteps=4, defaultStepSize=1)
        p = follow vf from 1@1 for 4
        ego = Object at p, facing p.heading
    """)
    assert tuple(ego.position) == pytest.approx((-1, 3))
    assert ego.heading == pytest.approx(math.radians(90))

## Region operators

# visible

def test_visible():
    scenario = compileScenic("""
        ego = Object at 100 @ 200, facing -45 deg,
                     with visibleDistance 10, with viewAngle 90 deg
        reg = RectangularRegion(100@205, 0, 10, 20)
        param p = Point in visible reg
    """)
    for i in range(30):
        p = sampleParamP(scenario, maxIterations=100)
        assert p.x >= 100
        assert p.y >= 200
        assert math.hypot(p.x - 100, p.y - 200) <= 10

def test_not_visible():
    scenario = compileScenic("""
        ego = Object at 100 @ 200, facing -45 deg,
                     with visibleDistance 30, with viewAngle 90 deg
        reg = RectangularRegion(100@200, 0, 10, 10)
        param p = Point in not visible reg
    """)
    ps = [sampleParamP(scenario, maxIterations=100) for i in range(50)]
    assert all(p.x <= 100 or p.y <= 200 for p in ps)
    assert any(p.x > 100 for p in ps)
    assert any(p.y > 200 for p in ps)
