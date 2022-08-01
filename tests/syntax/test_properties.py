
import pytest
import numpy as np

from scenic.core.errors import RuntimeParseError
from tests.utils import compileScenic, sampleEgoFrom

def test_position_wrong_type():
    with pytest.raises(RuntimeParseError):
        compileScenic('ego = new Object with position 4')

def test_position_oriented_point():
    sampleEgoFrom("""
        a = new OrientedPoint at 1@0
        b = new OrientedPoint at 0@1
        ego = new Object with position Uniform(a, b)
    """)

def test_position_numpy_types():
    ego = sampleEgoFrom("""
        import numpy as np
        ego = new Object with position np.single(3.4) @ np.single(7)
    """)
    assert tuple(ego.position) == pytest.approx((3.4, 7))

def test_heading_wrong_type():
    with pytest.raises(RuntimeParseError):
        compileScenic('ego = new Object with heading 4 @ 1')

def test_heading_numpy_types():
    ego = sampleEgoFrom("""
        import numpy as np
        ego = new Object with heading np.single(3.4)
    """)
    assert ego.heading == pytest.approx(3.4)

def test_left():
    ego = sampleEgoFrom("""
        other = new Object with width 4
        ego = new Object at other.left offset by 0@5
    """)
    assert tuple(ego.position) == pytest.approx((-2, 5))
