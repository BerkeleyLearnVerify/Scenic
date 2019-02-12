
import pytest

import scenic
from scenic import scenarioFromString as compileScenic
from tests.utils import sampleEgo

## Basic

def test_requirement():
    scenario = compileScenic(
        'ego = Object at (-10, 10) @ 0\n'
        'require ego.position.x >= 0'
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(100)]
    assert all(0 <= x <= 10 for x in xs)

def test_soft_requirement():
    scenario = compileScenic(
        'ego = Object at (-10, 10) @ 0\n'
        'require[0.9] ego.position.x >= 0'
    )
    xs = [sampleEgo(scenario, maxIterations=60).position.x for i in range(350)]
    count = sum(x >= 0 for x in xs)
    assert 255 <= count < 350
