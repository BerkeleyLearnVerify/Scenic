"""Tests for modular scenarios."""

import pytest

from scenic.core.dynamics import PreconditionViolation
from scenic.core.errors import RuntimeParseError, ScenicSyntaxError
from scenic.core.simulators import DummySimulator

from tests.utils import (compileScenic, sampleEgo, sampleEgoFrom, sampleScene,
                         sampleSceneFrom, sampleTrajectory)

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

def test_main_scenario():
    scene = sampleSceneFrom("""
        scenario Other():
            ego = Object at (10, 5)
        scenario Main():
            ego = Object at (1, 2)
    """)
    assert len(scene.objects) == 1
    assert tuple(scene.egoObject.position) == (1, 2)

def test_requirement():
    scenario = compileScenic("""
        scenario Main():
            setup:
                ego = Object with width Range(1, 3)
                require ego.width > 2
    """)
    ws = [sampleEgo(scenario, maxIterations=60).width for i in range(60)]
    assert all(2 < w <= 3 for w in ws)

# Preconditions and invariants

def test_top_level_precondition():
    scenario = compileScenic("""
        scenario Main():
            precondition: simulation().currentTime > 0
            setup:
                ego = Object
    """)
    sim = DummySimulator(timestep=1)
    scene = sampleScene(scenario)
    with pytest.raises(PreconditionViolation):
        sim.simulate(scene, maxSteps=1, raiseGuardViolations=True)

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

def test_choose_1():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do choose Sub(-1), Sub(3)
        scenario Sub(x):
            precondition: simulation().currentTime >= x
            setup:
                ego = Object at x @ 0
    """, scenario='Main')
    xs = [sampleTrajectory(scenario, maxSteps=1)[1][0][0] for i in range(30)]
    assert all(x == -1 for x in xs)

def test_choose_2():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do choose Sub(-1), Sub(-2), Sub(5)
        scenario Sub(x):
            precondition: simulation().currentTime >= x
            setup:
                ego = Object at x @ 0
    """, scenario='Main')
    xs = [sampleTrajectory(scenario, maxSteps=1)[1][0][0] for i in range(30)]
    assert all(x == -1 or x == -2 for x in xs)
    assert any(x == -1 for x in xs)
    assert any(x == -2 for x in xs)

def test_choose_deadlock():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do choose Sub(6), Sub(2)
        scenario Sub(x):
            precondition: simulation().currentTime >= x
            setup:
                ego = Object at x @ 0
    """, scenario='Main')
    scene = sampleScene(scenario)
    sim = DummySimulator(timestep=1)
    result = sim.simulate(scene, maxSteps=1)
    assert result is None

def test_shuffle_1():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do shuffle Sub(-1), Sub(1)
        scenario Sub(x):
            precondition: simulation().currentTime >= x
            setup:
                ego = Object at x @ 0
                terminate after 1
    """, scenario='Main')
    for i in range(30):
        trajectory = sampleTrajectory(scenario, maxSteps=3)
        assert len(trajectory) == 3
        assert len(trajectory[1]) == 1
        assert len(trajectory[2]) == 2
        assert trajectory[1][0] == (-1, 0)
        assert trajectory[2][1] == (1, 0)

def test_shuffle_2():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do shuffle Sub(1), Sub(3)
        scenario Sub(x):
            ego = Object at x @ 0
            terminate after 1
    """, scenario='Main')
    x1s = []
    for i in range(30):
        trajectory = sampleTrajectory(scenario, maxSteps=3)
        assert len(trajectory) == 3
        assert len(trajectory[1]) == 1
        assert len(trajectory[2]) == 2
        x1 = trajectory[1][0].x
        x2 = trajectory[2][1].x
        assert x1 == 1 or x1 == 3
        assert x2 == 1 or x2 == 3
        assert x1 != x2
        x1s.append(x1)
    assert any(x1 == 1 for x1 in x1s)
    assert any(x1 == 3 for x1 in x1s)

def test_shuffle_deadlock():
    scenario = compileScenic("""
        scenario Main():
            compose:
                do shuffle Sub(-1), Sub(2)
        scenario Sub(x):
            precondition: simulation().currentTime >= x
            setup:
                ego = Object at x @ 0
                terminate after 1
    """, scenario='Main')
    scene = sampleScene(scenario)
    sim = DummySimulator(timestep=1)
    result = sim.simulate(scene, maxSteps=2)
    assert result is None

# Scoping

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
