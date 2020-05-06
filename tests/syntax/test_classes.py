
import math
import pytest

from scenic import scenarioFromString as compileScenic
from scenic.syntax.translator import TokenParseError, ASTParseError, InterpreterParseError

def test_wrong_class_statement():
    with pytest.raises(TokenParseError):
        compileScenic(
            'constructor Foo(object):\n'
            '    pass'
        )
    with pytest.raises(TokenParseError):
        compileScenic(
            'import collections\n'
            'constructor Foo(collections.defaultdict):\n'
            '    pass'
        )

def test_old_constructor_statement():
    compileScenic(
        'constructor Foo:\n'
        '    blah: 19 @ -3\n'
        'ego = Foo with blah 12\n'
    )

def test_python_class():
    scenario = compileScenic(
        'class Foo(object):\n'
        '    def __init__(self, x):\n'
        '         self.x = x\n'
        'ego = Object with width Foo(4).x'
    )
    scene, iterations = scenario.generate(maxIterations=1)
    ego = scene.egoObject
    assert ego.width == 4

def test_invalid_attribute():
    with pytest.raises(InterpreterParseError):
        compileScenic(
            'class Foo:\n'
            '    blah[baloney_attr]: 4'
        )

def test_property_simple():
    scenario = compileScenic(
        'class Foo:\n'
        '    position: 3 @ 9\n'
        '    flubber: -12\n'
        'ego = Foo'
    )
    scene, iterations = scenario.generate(maxIterations=1)
    ego = scene.egoObject
    assert type(ego).__name__ == 'Foo'
    assert tuple(ego.position) == (3, 9)
    assert ego.flubber == -12

def test_property_inheritance():
    scenario = compileScenic(
        'class Foo:\n'
        '    flubber: -12\n'
        'class Bar(Foo):\n'
        '    flubber: 7\n'
        'ego = Bar'
    )
    scene, iterations = scenario.generate(maxIterations=1)
    ego = scene.egoObject
    assert type(ego).__name__ == 'Bar'
    assert ego.flubber == 7
