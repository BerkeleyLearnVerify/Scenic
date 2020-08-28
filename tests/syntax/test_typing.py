import pytest

from scenic.core.errors import ScenicSyntaxError
from tests.utils import compileScenic, sampleEgoFrom, sampleParamP, sampleParamPFrom

# Vectors

def test_tuple_as_vector():
    p = sampleParamPFrom("""
        ego = Object at 1 @ 2
        param p = distance to (-2, -2)
    """)
    assert p == pytest.approx(5)

def test_tuple_as_vector_2():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("""
            ego = Object at 1 @ 2
            param p = distance to (-2, -2, 0)
        """)

def test_list_as_vector():
    p = sampleParamPFrom("""
        ego = Object at 1 @ 2
        param p = distance to [-2, -2]
    """)
    assert p == pytest.approx(5)

def test_list_as_vector_2():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("""
            ego = Object at 1 @ 2
            param p = distance to [-2, -2, 0]
        """)
