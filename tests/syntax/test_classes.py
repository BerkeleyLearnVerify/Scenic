
import math
import pytest

from scenic.core.errors import TokenParseError, ASTParseError, RuntimeParseError
from tests.utils import compileScenic, sampleScene


def test_old_constructor_statement():
    with pytest.raises(SyntaxError):
        compileScenic("""
            constructor Foo:
                blah: (19, -3)
            ego = Foo with blah 12
        """)

def test_python_class():
    scenario = compileScenic("""
        class Foo(object):
            def __init__(self, x):
                 self.x = x
        ego = new Object with width Foo(4).x
    """)
    scene = sampleScene(scenario, maxIterations=1)
    ego = scene.egoObject
    assert ego.width == 4

def test_invalid_attribute():
    with pytest.raises(SyntaxError):
        compileScenic("""
            class Foo:\n
                blah[baloney_attr]: 4
        """)

def test_property_simple():
    scenario = compileScenic("""
        class Foo:
            position: (3, 9)
            flubber: -12
        ego = new Foo
    """)
    scene = sampleScene(scenario, maxIterations=1)
    ego = scene.egoObject
    assert type(ego).__name__ == 'Foo'
    assert tuple(ego.position) == (3, 9)
    assert ego.flubber == -12

def test_property_inheritance():
    scenario = compileScenic("""
        class Foo:
            flubber: -12
        class Bar(Foo):
            flubber: 7
        ego = new Bar
    """)
    scene = sampleScene(scenario, maxIterations=1)
    ego = scene.egoObject
    assert type(ego).__name__ == 'Bar'
    assert ego.flubber == 7

def test_isinstance_issubclass():
    scenario = compileScenic("""
        class Foo: pass
        ego = new Foo
        if isinstance(ego, Foo):
            other = new Object at (10, 0)
        if not isinstance(other, Foo):
            new Object at (20, 0)
        if issubclass(Foo, Point):
            new Object at (30, 0)
    """)
    scene = sampleScene(scenario)
    assert len(scene.objects) == 4
