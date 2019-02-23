
import math
import pytest

from scenic.syntax.translator import InterpreterParseError
from tests.utils import compileScenic, sampleEgoFrom, sampleParamPFrom

## Scalar operators

# relative heading

def test_relative_heading():
    p = sampleParamPFrom(
        'ego = Object facing 30 deg\n'
        'other = Object facing 65 deg, at 10@10\n'
        'param p = relative heading of other'
    )
    assert p == pytest.approx(math.radians(65 - 30))

def test_relative_heading_no_ego():
    with pytest.raises(InterpreterParseError):
        compileScenic('other = Object\n' 'ego = Object at 2@2, facing relative heading of other')

def test_relative_heading_from():
    ego = sampleEgoFrom('ego = Object facing relative heading of 70 deg from -10 deg')
    assert ego.heading == pytest.approx(math.radians(70 + 10))

# apparent heading

def test_apparent_heading():
    p = sampleParamPFrom(
        'ego = Object facing 30 deg\n'
        'other = Object facing 65 deg, at 10@10\n'
        'param p = apparent heading of other'
    )
    assert p == pytest.approx(math.radians(65 + 45))

def test_apparent_heading_no_ego():
    with pytest.raises(InterpreterParseError):
        compileScenic('other = Object\n' 'ego = Object at 2@2, facing apparent heading of other')

def test_apparent_heading_from():
    ego = sampleEgoFrom(
        'OP = OrientedPoint at 10@15, facing -60 deg\n'
        'ego = Object facing apparent heading of OP from 15@10')
    assert ego.heading == pytest.approx(math.radians(-60 - 45))

# distance

def test_distance():
    p = sampleParamPFrom(
        'ego = Object at 5@10\n'
        'other = Object at 7@-4\n'
        'param p = distance to other'
    )
    assert p == pytest.approx(math.hypot(7 - 5, -4 - 10))

def test_distance_no_ego():
    with pytest.raises(InterpreterParseError):
        compileScenic('other = Object\n' 'ego = Object at 2@2, facing distance to other')

def test_distance_from():
    ego = sampleEgoFrom('ego = Object with wobble distance from -3@2 to 4@5')
    assert ego.wobble == pytest.approx(math.hypot(4 - -3, 5 - 2))

# angle

def test_angle():
    p = sampleParamPFrom(
        'ego = Object facing 30 deg\n'
        'other = Object facing 65 deg, at 10@10\n'
        'param p = angle to other'
    )
    assert p == pytest.approx(math.radians(-45))

def test_angle_no_ego():
    with pytest.raises(InterpreterParseError):
        compileScenic('other = Object\n' 'ego = Object at 2@2, facing angle to other')

def test_angle_from():
    ego = sampleEgoFrom('ego = Object facing angle from 2@4 to 3@5')
    assert ego.heading == pytest.approx(math.radians(-45))

## Boolean operators

# can see

def test_point_can_see_vector():
    p = sampleParamPFrom(
        'ego = Object\n'
        'pt = Point at 10@20, with visibleDistance 5\n'
        'param p = tuple([pt can see 8@19, pt can see 10@26])'
    )
    assert p == (True, False)

def test_point_can_see_object():
    p = sampleParamPFrom(
        'ego = Object with width 10, with height 10\n'
        'other = Object at 35@10\n'
        'pt = Point at 15@10, with visibleDistance 15\n'
        'param p = tuple([pt can see ego, pt can see other])'
    )
    assert p == (True, False)

def test_oriented_point_can_see_vector():
    p = sampleParamPFrom(
        'ego = Object facing -45 deg, with visibleDistance 5, with viewAngle 20 deg\n'
        'param p = tuple([ego can see 2@2, ego can see 4@4, ego can see 1@0])'
    )
    assert p == (True, False, False)

def test_oriented_point_can_see_object():
    p = sampleParamPFrom(
        'ego = Object facing -45 deg, with visibleDistance 5, with viewAngle 20 deg\n'
        'other = Object at 4@4, with width 2, with height 2\n'
        'other2 = Object at 4@0, with requireVisible False\n'
        'param p = tuple([ego can see other, ego can see other2])'
    )
    assert p == (True, False)

# in

def test_point_in_region():
    p = sampleParamPFrom(
        'ego = Object\n'
        'reg = RectangularRegion(10@5, 0, 4, 2)\n'
        'ptA = Point at 11@4.5\n'
        'ptB = Point at 11@3.5\n'
        'param p = tuple([9@5.5 in reg, 9@7 in reg, ptA in reg, ptB in reg])'
    )
    assert p == (True, False, True, False)

def test_object_in_region():
    p = sampleParamPFrom(
        'reg = RectangularRegion(10@5, 0, 4, 2)\n'
        'ego = Object at 11.5@5.5, with width 0.25, with height 0.25\n'
        'other = Object at 9@4.5, with width 2.5\n'
        'param p = tuple([ego in reg, other in reg])'
    )
    assert p == (True, False)

## Heading operators

# at

def test_field_at_vector():
    ego = sampleEgoFrom(
        'vf = VectorField("Foo", lambda pos: (3 * pos.x) + pos.y)\n'
        'ego = Object facing (vf at 0.02 @ -1)'
    )
    assert ego.heading == pytest.approx((3 * 0.02) - 1)

# relative to

def test_heading_relative_to_field():
    ego = sampleEgoFrom(
        'vf = VectorField("Foo", lambda pos: 3 * pos.x)\n'
        'ego = Object at 0.07 @ 0, facing 0.5 relative to vf'
    )
    assert ego.heading == pytest.approx(0.5 + (3 * 0.07))

def test_field_relative_to_heading():
    ego = sampleEgoFrom(
        'vf = VectorField("Foo", lambda pos: 3 * pos.x)\n'
        'ego = Object at 0.07 @ 0, facing vf relative to 0.5'
    )
    assert ego.heading == pytest.approx(0.5 + (3 * 0.07))

def test_field_relative_to_field():
    ego = sampleEgoFrom(
        'vf = VectorField("Foo", lambda pos: 3 * pos.x)\n'
        'ego = Object at 0.07 @ 0, facing vf relative to vf'
    )
    assert ego.heading == pytest.approx(2 * (3 * 0.07))

def test_heading_relative_to_heading():
    ego = sampleEgoFrom('ego = Object facing 0.5 relative to -0.3')
    assert ego.heading == pytest.approx(0.5 - 0.3)

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
    ego = sampleEgoFrom(
        'vf = VectorField("Foo", lambda pos: 3 deg * pos.x)\n'
        'ego = Object at 15@7 offset along vf by 2@-3'
    )
    d = 1 / math.sqrt(2)
    assert tuple(ego.position) == pytest.approx((15 + 3*d + 2*d, 7 - 3*d + 2*d))

## Region operators

# visible

def test_visible():
    scenario = compileScenic(
        'ego = Object at 100 @ 200, facing -45 deg, \\\n'
        '             with visibleDistance 10, with viewAngle 90 deg\n'
        'reg = RectangularRegion(100@205, 0, 10, 10)\n'
        'param p = Point in visible reg'
    )
    for i in range(30):
        scene, iterations = scenario.generate(maxIterations=100)
        assert scene.params['p'].x >= 100
