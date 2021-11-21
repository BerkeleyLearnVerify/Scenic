"""Tests for modular scenarios."""

import pytest

from scenic.core.errors import RuntimeParseError, ScenicSyntaxError

from tests.utils import (compileScenic, sampleEgo, sampleEgoFrom, sampleTrajectory)

# Basics

def test_single_scenario():
    ego = sampleEgoFrom("""
        scenario Main():
            setup:
                ego = Object at (1, 2)
    """)
    assert tuple(ego.position) == (1, 2)

def test_simple_scenario():
    ego = sampleEgoFrom("""
        scenario Main():
            ego = Object at (1, 2)
    """)
    assert tuple(ego.position) == (1, 2)

def test_requirement():
    scenario = compileScenic("""
        scenario Main():
            setup:
                ego = Object with width Range(1, 3)
                require ego.width > 2
    """)
    ws = [sampleEgo(scenario, maxIterations=60).width for i in range(60)]
    assert all(2 < w <= 3 for w in ws)

# Composition

def test_parallel_composition():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do Sub(1), Sub(5)
        scenario Sub(x):
            ego = Object at x @ 0
    """, scenario='Main')
    trajectory = sampleTrajectory(scenario)
    assert len(trajectory) == 2
    assert len(trajectory[1]) == 2
    assert tuple(trajectory[1][0]) == (1, 0)
    assert tuple(trajectory[1][1]) == (5, 0)

def test_sequential_composition():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do Sub(1)
                do Sub(5)
        scenario Sub(x):
            ego = Object at x @ 0
            terminate after 1
    """, scenario='Main')
    trajectory = sampleTrajectory(scenario, maxSteps=3)
    assert len(trajectory) == 3
    assert len(trajectory[1]) == 1
    assert len(trajectory[2]) == 2
    assert tuple(trajectory[1][0]) == (1, 0)
    assert tuple(trajectory[2][0]) == (1, 0)
    assert tuple(trajectory[2][1]) == (5, 0)

def test_shared_scope_read():
    scenario = compileScenic("""
        scenario Main():
            setup:
                y = 3
            compose:
                do Sub(y)
        scenario Sub(x):
            ego = Object at x @ 0
    """, scenario='Main')
    trajectory = sampleTrajectory(scenario)
    assert len(trajectory) == 2
    assert len(trajectory[1]) == 1
    assert tuple(trajectory[1][0]) == (3, 0)

def test_shared_scope_write():
    scenario = compileScenic("""
        scenario Main():
            setup:
                y = 3
            compose:
                y = 4
                do Sub(y)
        scenario Sub(x):
            ego = Object at x @ 0
    """, scenario='Main')
    trajectory = sampleTrajectory(scenario)
    assert len(trajectory) == 2
    assert len(trajectory[1]) == 1
    assert tuple(trajectory[1][0]) == (4, 0)

def test_shared_scope_del():
    scenario = compileScenic("""
        scenario Main():
            setup:
                y = 3
            compose:
                del y
                do Sub()
        scenario Sub():
            ego = Object
    """, scenario='Main')
    sampleTrajectory(scenario)

def test_independent_requirements():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                wait
                self.position += DiscreteRange(0, 1) @ 0
        scenario Main():
            compose:
                do Sub(0, 1), Sub(2, 3)
        scenario Sub(start, dest):
            ego = Object at start @ 0, with behavior Foo
            require eventually ego.position.x >= dest
    """, scenario='Main')
    for i in range(30):
        trajectory = sampleTrajectory(scenario, maxSteps=2, maxIterations=100)
        assert tuple(trajectory[2][0]) == (1, 0)
        assert tuple(trajectory[2][1]) == (3, 0)
