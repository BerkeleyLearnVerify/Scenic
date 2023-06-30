import numpy as np
import pytest

from scenic.core.errors import SpecifierError
from tests.utils import compileScenic, sampleEgoFrom


def test_position_wrong_type():
    with pytest.raises(TypeError):
        compileScenic("ego = new Object with position 4")


def test_position_oriented_point():
    sampleEgoFrom(
        """
        a = new OrientedPoint at 1@0
        b = new OrientedPoint at 0@1
        ego = new Object with position Uniform(a, b)
    """
    )


def test_position_numpy_types():
    ego = sampleEgoFrom(
        """
        import numpy as np
        ego = new Object with position np.single(3.4) @ np.single(7)
    """
    )
    assert tuple(ego.position) == pytest.approx((3.4, 7, 0))


def test_yaw_wrong_type():
    with pytest.raises(TypeError):
        compileScenic("ego = new Object with yaw 4 @ 1")


def test_yaw_numpy_types():
    ego = sampleEgoFrom(
        """
        import numpy as np
        ego = new Object with yaw np.single(3.1)
    """
    )
    assert ego.yaw == pytest.approx(3.1)


def test_left():
    ego = sampleEgoFrom(
        """
        other = new Object with width 4
        ego = new Object at other.left offset by 0@5
    """
    )
    assert tuple(ego.position) == pytest.approx((-2, 5, 0))


def test_right():
    ego = sampleEgoFrom(
        """
        other = new Object with width 4
        ego = new Object at other.right offset by 0@5
    """
    )
    assert tuple(ego.position) == pytest.approx((2, 5, 0))


def test_front():
    ego = sampleEgoFrom(
        """
        other = new Object with length 4
        ego = new Object at other.front offset by 0@5
    """
    )
    assert tuple(ego.position) == pytest.approx((0, 7, 0))


def test_back():
    ego = sampleEgoFrom(
        """
        other = new Object with length 4
        ego = new Object at other.back offset by 0@-5
    """
    )
    assert tuple(ego.position) == pytest.approx((0, -7, 0))


def test_frontLeft():
    ego = sampleEgoFrom(
        """
        other = new Object with length 4, with width 2
        ego = new Object at other.frontLeft offset by 0@5
    """
    )
    assert tuple(ego.position) == pytest.approx((-1, 7, 0))


def test_frontRight():
    ego = sampleEgoFrom(
        """
        other = new Object with length 4, with width 2
        ego = new Object at other.frontRight offset by 0@5
    """
    )
    assert tuple(ego.position) == pytest.approx((1, 7, 0))


def test_backLeft():
    ego = sampleEgoFrom(
        """
        other = new Object with length 4, with width 2
        ego = new Object at other.backLeft offset by 0@-5
    """
    )
    assert tuple(ego.position) == pytest.approx((-1, -7, 0))


def test_backRight():
    ego = sampleEgoFrom(
        """
        other = new Object with length 4, with width 2
        ego = new Object at other.backRight offset by 0@-5
    """
    )
    assert tuple(ego.position) == pytest.approx((1, -7, 0))


def test_heading_set_directly():
    with pytest.raises(SpecifierError):
        compileScenic("ego = new Object with heading 4")
