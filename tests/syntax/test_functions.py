
import pytest

from tests.utils import compileScenic, sampleEgoFrom

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
