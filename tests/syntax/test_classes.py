
import math
import pytest

from scenic import scenarioFromString as compileScenic
from scenic.syntax.translator import ASTParseError, InterpreterParseError

def test_wrong_class_statement():
    with pytest.raises(ASTParseError):
        compileScenic(
            'class Foo(Point):\n'
            '    pass'
        )
    with pytest.raises(ASTParseError):
        compileScenic(
            'constructor Foo:\n'
            '    pass\n'
            'class Bar(Foo):\n'
            '    pass'
        )

def test_invalid_attribute():
    with pytest.raises(InterpreterParseError):
        compileScenic(
            'constructor Foo:\n'
            '    blah[baloney_attr]: 4'
        )

def test_property_simple():
    scenario = compileScenic(
        'constructor Foo:\n'
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
        'constructor Foo:\n'
        '    flubber: -12\n'
        'constructor Bar(Foo):\n'
        '    flubber: 7\n'
        'ego = Bar'
    )
    scene, iterations = scenario.generate(maxIterations=1)
    ego = scene.egoObject
    assert type(ego).__name__ == 'Bar'
    assert ego.flubber == 7
