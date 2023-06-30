import pytest

from tests.utils import compileScenic, sampleEgoFrom

# Built-in functions


def test_max_min():
    ego = sampleEgoFrom(
        """
        a = Range(0, 1)
        b = Range(0, 1)
        ego = new Object with foo max(a, b), with bar min(a, b)
    """
    )
    assert ego.foo >= ego.bar
    ego = sampleEgoFrom("ego = new Object with foo min([], default=Range(1,2))")
    assert 1 <= ego.foo <= 2
    ego = sampleEgoFrom("ego = new Object with foo min(1, 2, key=lambda x: -x)")
    assert ego.foo == 2


def test_str():
    ego = sampleEgoFrom("ego = new Object with foo str(Range(12, 17))")
    assert isinstance(ego.foo, str)
    assert 12 <= float(ego.foo) <= 17


def test_float():
    ego = sampleEgoFrom('ego = new Object with foo float(Uniform("1.5", "-0.5"))')
    assert isinstance(ego.foo, float)
    assert float(ego.foo) in (1.5, -0.5)


def test_int():
    ego = sampleEgoFrom("ego = new Object with foo int(Range(12, 14.9))")
    assert isinstance(ego.foo, int)
    assert ego.foo in (12, 13, 14)


def test_round():
    ego = sampleEgoFrom("ego = new Object with foo round(Range(12.51, 13.4))")
    assert isinstance(ego.foo, int)
    assert ego.foo == 13
    ego = sampleEgoFrom("ego = new Object with foo round(Range(12.5, 13.4), ndigits=1)")
    assert isinstance(ego.foo, float)
    assert ego.foo - round(ego.foo, ndigits=1) == pytest.approx(0)


# Iterable and dictionary unpacking


def test_unpacking():
    ego = sampleEgoFrom(
        """
        def func(*args, **kwargs):
            return [args, kwargs]
        ego = new Object with foo func(*[1,2,3], func=4)
    """
    )
    assert ego.foo == [(1, 2, 3), {"func": 4}]


def test_unpacking_distribution():
    ego = sampleEgoFrom(
        """
        def func(x, y):
            return [y, x]
        pairs = Uniform([1,2], [3,4])
        ego = new Object with foo func(*pairs)
    """
    )
    assert ego.foo[0] > ego.foo[1]


def test_unpacking_distribution_2():
    with pytest.raises(TypeError):
        sampleEgoFrom("ego = new Object with foo max(*Range(1,2))")
