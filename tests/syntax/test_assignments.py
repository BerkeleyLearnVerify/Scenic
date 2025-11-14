# Minimal semantics tests for multiple assignment and LHS unpacking.

import pytest

from tests.utils import compileScenic, sampleEgoActions


def test_multiple_assignment_swap():
    scenario = compileScenic(
        """
        behavior Foo():
            a = 1
            b = 2
            a, b = b, a
            take a
            take b
        ego = new Object with behavior Foo
        """
    )
    actions = sampleEgoActions(scenario, maxSteps=2)
    assert actions == [2, 1]


def test_lhs_tuple_unpack():
    scenario = compileScenic(
        """
        behavior Foo():
            x, y = (3, 4)
            take x
            take y
        ego = new Object with behavior Foo
        """
    )
    assert sampleEgoActions(scenario, maxSteps=2) == [3, 4]


def test_unpack_from_distribution_pair():
    scenario = compileScenic(
        """
        behavior Foo():
            pair = Uniform((0, 1), (1, 0))
            x, y = pair
            take x
            take y
        ego = new Object with behavior Foo
        """
    )
    outs = [tuple(sampleEgoActions(scenario, maxSteps=2)) for _ in range(60)]
    assert {(0, 1), (1, 0)} <= set(outs)


@pytest.mark.parametrize(
    "rhs",
    ["(1, 2, 3)", "(1,)"],  # too many; not enough
)
def test_unpack_arity_mismatch_exec_raises(rhs):
    scenario = compileScenic(
        f"""
        behavior Foo():
            x, y = {rhs}
            take x
        ego = new Object with behavior Foo
    """
    )
    with pytest.raises(ValueError):
        sampleEgoActions(scenario, maxSteps=1)
