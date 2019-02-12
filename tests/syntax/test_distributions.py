
import pytest
import math
import random

from scenic.syntax.translator import InterpreterParseError
from tests.utils import compileScenic, sampleEgo

## Options and Uniform

def test_options_empty_domain():
    with pytest.raises(InterpreterParseError):
        compileScenic('x = Options([])')
    with pytest.raises(InterpreterParseError):
        compileScenic('x = Options({})')
    with pytest.raises(InterpreterParseError):
        compileScenic('x = Uniform()')

def test_options_invalid_weight():
    with pytest.raises(InterpreterParseError):
        compileScenic('x = Options({0: 1, 1: -2})')
    with pytest.raises(InterpreterParseError):
        compileScenic('x = Options({0: 1, 1: []})')
    with pytest.raises(InterpreterParseError):
        compileScenic('x = Options({0: 1, 1: (3, 5)})')

def test_uniform_interval_wrong_type():
    with pytest.raises(InterpreterParseError):
        compileScenic('x = ([], 4)')
    with pytest.raises(InterpreterParseError):
        compileScenic('x = (-10, [])')

def test_uniform_interval():
    scenario = compileScenic('ego = Object at (100, 200) @ 0')
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(100 <= x <= 200 for x in xs)
    assert any(x < 150 for x in xs)
    assert any(150 < x for x in xs)

def test_uniform_discrete():
    scenario = compileScenic('ego = Object at Uniform(1, 2, 3.4) @ 0')
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(x == 1 or x == 2 or x == 3.4 for x in xs)
    assert any(x == 1 for x in xs)
    assert any(x == 2 for x in xs)
    assert any(x == 3.4 for x in xs)

def test_options():
    scenario = compileScenic('ego = Object at Options({0: 1, 1: 9}) @ 0')
    xs = [sampleEgo(scenario).position.x for i in range(400)]
    assert all(x == 0 or x == 1 for x in xs)
    assert sum(xs) >= 300

## Functions, methods, attributes, operators

def test_function():
    scenario = compileScenic('ego = Object at sin(Uniform(45 deg, 90 deg)) @ 0')
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    valA, valB = (pytest.approx(math.sin(math.radians(a))) for a in (45, 90))
    assert all(x == valA or x == valB for x in xs)
    assert any(x == valA for x in xs)
    assert any(x == valB for x in xs)

def test_method():
    scenario = compileScenic(
        'field = VectorField("Foo", lambda pos: pos[1])\n'
        'ang = field[0 @ (100, 200)]\n'
        'ego = Object facing ang'
    )
    angles = [sampleEgo(scenario).heading for i in range(100)]
    assert all(100 <= x <= 200 for x in angles)
    assert any(x < 150 for x in angles)
    assert any(150 < x for x in angles)

def test_attribute():
    scenario = compileScenic(
        'place = Uniform(1 @ 1, 2 @ 4, 3 @ 9)\n'
        'ego = Object at place.x @ place.y'
    )
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(x == 1 or x == 2 or x == 3 for x in xs)
    assert any(x == 1 for x in xs)
    assert any(x == 2 for x in xs)
    assert any(x == 3 for x in xs)

def test_operator():
    scenario = compileScenic('ego = Object at -(100 + (0, 100)) @ 0')
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(-200 <= x <= -100 for x in xs)
    assert any(x < -150 for x in xs)
    assert any(-150 < x for x in xs)

## Vectors

def test_vector_operator():
    scenario = compileScenic('ego = Object at (-3, 3) @ 0 + (100, 110) @ 0')
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(97 <= x <= 113 for x in xs)
    assert any(x < 105 for x in xs)
    assert any(105 < x for x in xs)

## Reproducibility

def test_reproducibility():
    scenario = compileScenic(
        'ego = Object\n'
        'Object offset by 0@3, facing (0, 360) deg\n'
        'Object offset by 0@6, facing (0, 360) deg\n'
        'param foo = Uniform(1, 4, 9, 16, 25, 36)\n'
        'x = (0, 1)\n'
        'require x > 0.8'
    )
    for i in range(10):
        seed = random.randint(0, 100000)
        random.seed(seed)
        baseScene, baseIterations = scenario.generate(maxIterations=200)
        for j in range(20):
            random.seed(seed)
            scene, iterations = scenario.generate(maxIterations=200)
            assert len(scene.objects) == len(baseScene.objects)
            for obj, baseObj in zip(scene.objects, baseScene.objects):
                assert obj.heading == baseObj.heading
            assert scene.params['foo'] == baseScene.params['foo']
            assert iterations == baseIterations
