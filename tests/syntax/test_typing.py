import pytest

from tests.utils import compileScenic, sampleEgoFrom, sampleParamP, sampleParamPFrom

# Vectors


def test_tuple_as_vector():
    p = sampleParamPFrom(
        """
        ego = new Object at 1 @ 2
        param p = distance to (-2, -2)
    """
    )
    assert p == pytest.approx(5)


def test_tuple_as_vector_2():
    p = sampleParamPFrom(
        """
        ego = new Object at 1 @ 2
        param p = distance to (-2, -2, 12)
    """
    )
    assert p == pytest.approx(13)


def test_tuple_as_vector_3():
    with pytest.raises(TypeError):
        compileScenic(
            """
            ego = new Object at 1 @ 2
            param p = distance to (-2, -2, 0, 4)
        """
        )


def test_list_as_vector():
    p = sampleParamPFrom(
        """
        ego = new Object at 1 @ 2
        param p = distance to [-2, -2]
    """
    )
    assert p == pytest.approx(5)


def test_list_as_vector_2():
    p = sampleParamPFrom(
        """
        ego = new Object at 1 @ 2
        param p = distance to [-2, -2, 12]
    """
    )
    assert p == pytest.approx(13)


def test_list_as_vector_3():
    with pytest.raises(TypeError):
        compileScenic(
            """
            ego = new Object at 1 @ 2
            param p = distance to [-2, -2, 0, 6]
        """
        )
