
import pytest

from tests.utils import compileScenic, sampleEgoFrom

# Built-in functions

def test_max_min():
    ego = sampleEgoFrom("""
        a = Range(0, 1)
        b = Range(0, 1)
        ego = Object with foo max(a, b), with bar min(a, b)
    """)
    assert ego.foo >= ego.bar
    ego = sampleEgoFrom('ego = Object with foo min([], default=Range(1,2))')
    assert 1 <= ego.foo <= 2
    ego = sampleEgoFrom('ego = Object with foo min(1, 2, key=lambda x: -x)')
    assert ego.foo == 2

def test_str():
    ego = sampleEgoFrom('ego = Object with foo str(Range(12, 17))')
    assert isinstance(ego.foo, str)
    assert 12 <= float(ego.foo) <= 17

# Iterable and dictionary unpacking

def test_unpacking():
    ego = sampleEgoFrom("""
        def func(*args, **kwargs):
            return [args, kwargs]
        ego = Object with foo func(*[1,2,3], func=4)
    """)
    assert ego.foo == [(1, 2, 3), {'func': 4}]

def test_unpacking_distribution():
    ego = sampleEgoFrom("""
        def func(x, y):
            return [y, x]
        pairs = Uniform([1,2], [3,4])
        ego = Object with foo func(*pairs)
    """)
    assert ego.foo[0] > ego.foo[1]

def test_unpacking_distribution_2():
    with pytest.raises(TypeError):
        sampleEgoFrom('ego = Object with foo max(*Range(1,2))')
