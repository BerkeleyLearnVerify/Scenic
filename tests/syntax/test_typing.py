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


# isinstance Tests
def test_isinstance_str():
    p = sampleParamPFrom(
        """
        from scenic.core.type_support import isA
        param p = str(1)
        assert isinstance(globalParameters.p, str)
        assert isA(globalParameters.p, str)
        """
    )
    assert isinstance(p, str)

    p = sampleParamPFrom(
        """
        from scenic.core.type_support import isA
        param p = str(Range(0,2))
        assert isA(globalParameters.p, str)
        """
    )
    assert isinstance(p, str)


def test_isinstance_float():
    p = sampleParamPFrom(
        """
        from scenic.core.type_support import isA
        param p = float(1)
        assert isinstance(globalParameters.p, float)
        assert isA(globalParameters.p, float)
        """
    )
    assert isinstance(p, float)

    p = sampleParamPFrom(
        """
        from scenic.core.type_support import isA
        param p = float(Range(0,2))
        assert isA(globalParameters.p, float)
        """
    )
    assert isinstance(p, float)


def test_isinstance_int():
    p = sampleParamPFrom(
        """
        from scenic.core.type_support import isA
        param p = int(1.5)
        assert isinstance(globalParameters.p, int)
        assert isA(globalParameters.p, int)
        """
    )
    assert isinstance(p, int)

    p = sampleParamPFrom(
        """
        from scenic.core.type_support import isA
        param p = int(Range(0,2))
        assert isA(globalParameters.p, int)
        """
    )
    assert isinstance(p, int)
