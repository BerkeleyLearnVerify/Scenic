# Minimal semantics tests for multiple assignment and LHS unpacking.

import pytest

from tests.utils import sampleParamPFrom


def test_multiple_assignment_swap():
    p = sampleParamPFrom(
        """
        a = 1
        b = 2
        a, b = b, a
        param p = (a, b)
        """
    )
    assert p == (2, 1)


def test_lhs_tuple_unpack():
    p = sampleParamPFrom(
        """
        x, y = (3, 4)
        param p = (x, y)
        """
    )
    assert p == (3, 4)


def test_unpack_from_distribution_pair():
    code = """
        pair = (Uniform(0, 1), Uniform(0, 1))
        x, y = pair
        param p = (x, y)
    """
    outs = [sampleParamPFrom(code) for _ in range(20)]
    assert len(set(outs)) > 1
    for x, y in outs:
        assert 0 <= x <= 1
        assert 0 <= y <= 1


@pytest.mark.parametrize(
    "rhs",
    ["(1, 2, 3)", "(1,)"],  # too many; not enough
)
def test_unpack_arity_mismatch_exec_raises(rhs):
    with pytest.raises(ValueError):
        sampleParamPFrom(
            f"""
            x, y = {rhs}
            param p = x
            """
        )
