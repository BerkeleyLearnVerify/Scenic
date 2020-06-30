
import pytest

from scenic.core.utils import RuntimeParseError

from tests.utils import (compileScenic, sampleScene, sampleActions, sampleEgoActions,
                         sampleEgoActionsFromScene, checkErrorLineNumber)

## Behaviors

# Arguments

def test_behavior_random_argument():
    scenario = compileScenic(
        'behavior Foo(arg):\n'
        '    take arg\n'
        'ego = Object with behavior Foo((10, 25))'
    )
    scene = sampleScene(scenario)
    actions1 = sampleEgoActionsFromScene(scene)
    assert 10 <= actions1[0] <= 25
    actions2 = sampleEgoActionsFromScene(scene)
    assert actions1 == actions2
    scene2 = sampleScene(scenario)
    actions3 = sampleEgoActionsFromScene(scene2)
    assert actions1 != actions3

# Termination

def test_behavior_end_early():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    take 5\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (5, None, None)

# Reuse

def test_behavior_reuse():
    scenario = compileScenic(
        'behavior Foo(arg):\n'
        '    take arg\n'
        'ego = Object with behavior Foo(3)\n'
        'Object at 10@10, with behavior Foo(5)'
    )
    actions = sampleActions(scenario)
    assert tuple(actions) == ((3, 5),)

def test_behavior_reuse_2():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    take (-10, 10)\n'
        'ego = Object with behavior Foo\n'
        'Object at 10@10, with behavior Foo'
    )
    actions = sampleActions(scenario)
    assert len(actions) == 1
    action1, action2 = actions[0]
    assert action1 != action2

# Nesting (sub-behaviors)

def test_behavior_nesting():
    scenario = compileScenic(
        'behavior Foo(a):\n'
        '    take a\n'
        '    take a\n'
        'behavior Bar():\n'
        '    take 1\n'
        '    Foo(2)\n'
        '    take 3\n'
        'ego = Object with behavior Bar\n'
    )
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 2, 2, 3)

# Interrupts

def test_interrupt():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    try:\n'
        '        while True:\n'
        '            take 1\n'
        '    interrupt when simulation().currentTime % 3 == 2:\n'
        '        take 2\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=6)
    assert tuple(actions) == (1, 1, 2, 1, 1, 2)

def test_interrupt_first():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    try:\n'
        '        while True:\n'
        '            take 1\n'
        '    interrupt when simulation().currentTime == 0:\n'
        '        take 2\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (2, 1, 1)

def test_interrupt_priority():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    try:\n'
        '        while True:\n'
        '            take 1\n'
        '    interrupt when simulation().currentTime <= 1:\n'
        '        take 2\n'
        '    interrupt when simulation().currentTime == 0:\n'
        '        take 3\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (3, 2, 1)

def test_interrupt_interrupted():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    try:\n'
        '        while True:\n'
        '            take 1\n'
        '    interrupt when simulation().currentTime <= 1:\n'
        '        take 2\n'
        '        take 3\n'
        '    interrupt when simulation().currentTime == 1:\n'
        '        take 4\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (2, 4, 3, 1, 1)

def test_interrupt_actionless():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    i = 0\n'
        '    try:\n'
        '        for i in range(3):\n'
        '            take 1\n'
        '    interrupt when i == 1:\n'
        '        i = 2\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (1, 1, 1, None, None)

def test_interrupt_unassigned_local():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    try:\n'
        '        i = 0\n'
        '        take 1\n'
        '    interrupt when i == 1:\n'
        '        i = 2\n'
        'ego = Object with behavior Foo'
    )
    with pytest.raises(NameError) as exc_info:
        sampleEgoActions(scenario, maxSteps=1)
    checkErrorLineNumber(5, exc_info)

def test_interrupt_guard_subbehavior():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    try:\n'
        '        take 1\n'
        '    interrupt when Foo():\n'
        '        wait\n'
        'ego = Object with behavior Foo'
    )
    with pytest.raises(RuntimeParseError):
        sampleEgoActions(scenario, maxSteps=1)
