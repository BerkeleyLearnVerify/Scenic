
import pytest

from scenic.core.errors import RuntimeParseError

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

def test_behavior_random_argument_list():
    scenario = compileScenic(
        'behavior Foo(arg):\n'
        '    take arg[1]\n'
        'ego = Object with behavior Foo([-5, (10, 25)])'
    )
    scene = sampleScene(scenario)
    actions1 = sampleEgoActionsFromScene(scene)
    assert 10 <= actions1[0] <= 25
    actions2 = sampleEgoActionsFromScene(scene)
    assert actions1 == actions2
    scene2 = sampleScene(scenario)
    actions3 = sampleEgoActionsFromScene(scene2)
    assert actions1 != actions3

# Globals

def test_behavior_globals_read():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take other.position.x
        ego = Object with behavior Foo
        other = Object at (10, 20) @ 15
    """)
    actions1 = sampleEgoActions(scenario, maxSteps=2)
    assert len(actions1) == 2
    assert 10 <= actions1[0] <= 20
    assert actions1[0] == actions1[1]
    actions2 = sampleEgoActions(scenario, maxSteps=1)
    assert len(actions2) == 1
    assert actions2[0] != actions1[0]

def test_behavior_globals_read_module(runLocally):
    with runLocally():
        scenario = compileScenic("""
            import helper4
            behavior Foo():
                while True:
                    take helper4.foo
            ego = Object with behavior Foo
        """)
        actions1 = sampleEgoActions(scenario, maxSteps=2)
        assert len(actions1) == 2
        assert 0 <= actions1[0] <= 1
        assert actions1[0] == actions1[1]
        actions2 = sampleEgoActions(scenario, maxSteps=1)
        assert len(actions2) == 1
        assert actions2[0] != actions1[0]

def test_behavior_globals_read_list():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take foo[1]
        ego = Object with behavior Foo
        foo = [5, (10, 20)]
    """)
    actions1 = sampleEgoActions(scenario, maxSteps=2)
    assert len(actions1) == 2
    assert 10 <= actions1[0] <= 20
    assert actions1[0] == actions1[1]
    actions2 = sampleEgoActions(scenario, maxSteps=1)
    assert len(actions2) == 1
    assert actions2[0] != actions1[0]

def test_behavior_globals_write():
    scenario = compileScenic("""
        glob = 0
        behavior Foo():
            global glob
            while True:
                take 0
                glob = 2
        behavior Bar():
            while True:
                take (glob < 1)
        other = Object with behavior Foo
        ego = Object at 10@10, with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert len(actions) == 3
    assert actions[0] == True
    assert actions[2] == False

# Implicit self

def test_behavior_self():
    scenario = compileScenic("""
        behavior Foo():
            take self.bar
        ego = Object with behavior Foo, with bar 3
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (3,)

def test_behavior_lazy():
    scenario = compileScenic("""
        vf = VectorField("Foo", lambda pos: pos.x)
        behavior Foo():
            take 1 relative to vf
        ego = Object at 0.5@0, with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (pytest.approx(1.5),)

def test_behavior_lazy_nested():
    scenario = compileScenic("""
        vf = VectorField("Foo", lambda pos: pos.x)
        behavior Foo():
            Bar()
            take -1 relative to vf
        behavior Bar():
            take 1 relative to vf
        behavior Baz():
            Bar(); Bar()
        Object at -10@0, with behavior Baz
        ego = Object at 0.5@0, with behavior Foo
    """)
    actions = sampleActions(scenario, maxSteps=2)
    assert tuple(actions) == (pytest.approx((1.5, -9)), pytest.approx((-0.5, -9)))

# Termination

def test_behavior_end_early():
    scenario = compileScenic("""
        behavior Foo():
            take 5
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (5, None, None)

def test_terminate():
    scenario = compileScenic("""
        behavior Foo():
            take 1
            terminate
            take 2
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (1,)

def test_terminate_when():
    scenario = compileScenic("""
        flag = False
        behavior Foo():
            global flag
            take 1
            flag = True
            take 2
        ego = Object with behavior Foo
        terminate when flag
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (1,)

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

def test_behavior_calls():
    """Ordinary function calls inside behaviors should still work."""
    scenario = compileScenic("""
        def func(a, *b, c=0, d=1, **e):
            return [a, len(b), c, d, len(e)]
        behavior Foo():
            take func(4, 5, 6, blah=4, c=10)
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == ([4, 2, 10, 1, 1],)

def test_behavior_calls_nested():
    """Nested function calls inside behaviors should still work."""
    scenario = compileScenic("""
        def funcA(x):
            return x+1
        def funcB(x):
            return x*2
        behavior Foo():
            take funcA(funcB(5))
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (11,)

def test_behavior_calls_side_effects():
    scenario = compileScenic("""
        x = 0
        def func():
            global x
            x += 1
            return x
        behavior Foo():
            while True:
                take func()
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 2, 3, 4)

# Requirements

def test_behavior_require():
    scenario = compileScenic("""
        behavior Foo():
            x = (-1, 1)
            require x > 0
            take x
        ego = Object with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=1, maxIterations=30)
        assert actions[0] > 0

def test_behavior_require_call():
    scenario = compileScenic("""
        behavior Foo():
            x = Uniform([], [1, 2])
            require len(x) > 0
            take x
        ego = Object with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=1, maxIterations=30)
        assert actions[0] == [1, 2]

## Monitors

def test_monitor():
    scenario = compileScenic("""
        monitor Monitor:
            while True:
                if ego.blah >= 3:
                    terminate
                wait
        behavior Foo():
            while True:
                take self.blah
                self.blah += 1
        ego = Object with blah 0, with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (0, 1, 2, 3)

## Interrupts

# Basic

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

# Exception handling

def test_interrupt_no_handlers():
    """Test a try-except statement that isn't actually a try-interrupt statement."""
    scenario = compileScenic("""
        behavior Foo():
            try:
                for i in range(3):
                    take 1
                    raise Exception
            except Exception:
                take 2
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (1, 2, None)

def test_interrupt_except():
    scenario = compileScenic("""
        behavior Foo():
            try:
                for i in range(3):
                    take 1
            interrupt when simulation().currentTime == 1:
                take 2
                raise Exception
                take 3
            except Exception:
                take 4
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 2, 4, None)

def test_interrupt_except_else():
    scenario = compileScenic("""
        behavior Foo():
            try:
                for i in range(3):
                    take 1
            interrupt when simulation().currentTime == 1:
                take 2
                take 3
            except Exception:
                take 4
            else:
                take 5
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=7)
    assert tuple(actions) == (1, 2, 3, 1, 1, 5, None)

# Nesting

def test_interrupt_nested():
    scenario = compileScenic("""
        behavior Foo():
            try:
                for i in range(3):
                    take 1
            interrupt when 1 <= simulation().currentTime <= 2:
                take 2
        behavior Bar():
            try:
                Foo()
            interrupt when simulation().currentTime == 1:
                take 3
        ego = Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=6)
    assert tuple(actions) == (1, 3, 2, 1, 1, None)

def test_interrupt_nested_2():
    scenario = compileScenic("""
        behavior Foo():
            try:
                for i in range(3):
                    take 1
            interrupt when simulation().currentTime == 1:
                take 2
                take 3
        behavior Bar():
            try:
                Foo()
            interrupt when simulation().currentTime == 2:
                take 4
        ego = Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=7)
    assert tuple(actions) == (1, 2, 4, 3, 1, 1, None)

def test_interrupt_nested_3():
    scenario = compileScenic("""
        behavior Bar():
            try:
                try:
                    for i in range(3):
                        take 1
                interrupt when 1 <= simulation().currentTime <= 2:
                    take 2
            interrupt when simulation().currentTime == 1:
                take 3
        ego = Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=6)
    assert tuple(actions) == (1, 3, 2, 1, 1, None)

# Control flow statements

def test_interrupt_break():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                try:
                    for i in range(3):
                        take 1
                interrupt when simulation().currentTime == 2:
                    take 2
                    break
                    take 3
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 1, 2, None)

def test_interrupt_break_2():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                try:
                    for i in range(3):
                        take 1
                interrupt when simulation().currentTime == 1:
                    for i in range(3):
                        take 2
                        break
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 2, 1, 1)

def test_interrupt_continue():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take 4
                try:
                    for i in range(2):
                        take 1
                interrupt when simulation().currentTime == 2:
                    take 2
                    continue
                    take 3
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=7)
    assert tuple(actions) == (4, 1, 2, 4, 1, 1, 4)

def test_interrupt_continue_2():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                try:
                    for i in range(3):
                        take 1
                interrupt when simulation().currentTime == 1:
                    for i in range(3):
                        if i == 0:
                            continue
                        take 2
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (1, 2, 2, 1, 1)

def test_interrupt_abort():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take 3
                try:
                    for i in range(3):
                        take 1
                interrupt when simulation().currentTime == 2:
                    for i in range(3):
                        take 2
                        abort
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=8)
    assert tuple(actions) == (3, 1, 2, 3, 1, 1, 1, 3)

# Errors

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
