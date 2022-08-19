
import sys

import pytest

from scenic.core.errors import RuntimeParseError, ScenicSyntaxError
from scenic.core.simulators import TerminationType

from tests.utils import (compileScenic, sampleScene, sampleActions, sampleActionsFromScene,
                         sampleEgoActions, sampleEgoActionsFromScene, sampleResult,
                         sampleResultOnce, checkErrorLineNumber)

## Dynamic state

def test_dynamic_property():
    scenario = compileScenic("""
        behavior Foo():
            for i in range(3):
                self.position = self.position + 1@0
                wait
        ego = new Object with behavior Foo
        terminate when ego.position.x >= 3
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert len(actions) == 3

def test_dynamic_derived_property():
    scenario = compileScenic("""
        behavior Foo():
            for i in range(3):
                self.position = self.position + 0@1
                wait
        ego = new Object with behavior Foo
        terminate when ego.left.position.y >= 3
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert len(actions) == 3

## Behaviors

# Basic

def test_behavior_actions():
    scenario = compileScenic("""
        behavior Foo():
            take 3
            take 5
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=2)
    assert tuple(actions) == (3, 5)

def test_behavior_multiple_actions():
    scenario = compileScenic("""
        behavior Foo():
            take 1, 4, 9
            take 5
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=2, singleAction=False)
    assert tuple(actions) == ((1, 4, 9), (5,))

def test_behavior_tuple_actions():
    scenario = compileScenic("""
        behavior Foo():
            take (1, 4, 9)
            take 5
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=2, singleAction=False)
    assert tuple(actions) == ((1, 4, 9), (5,))

def test_behavior_list_actions():
    scenario = compileScenic("""
        behavior Foo():
            take [1, 4, 9]
            take 5
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=2, singleAction=False)
    assert tuple(actions) == ((1, 4, 9), (5,))

# Various errors

def test_behavior_no_actions():
    with pytest.raises(ScenicSyntaxError):
        scenario = compileScenic("""
            behavior Foo():
                take 1
            behavior Bar():
                Foo()   # forgot to use 'do'
            ego = new Object with behavior Bar
        """)

# Arguments

def test_behavior_random_argument():
    scenario = compileScenic(
        'behavior Foo(arg):\n'
        '    take arg\n'
        'ego = new Object with behavior Foo(Range(10, 25))'
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
        'ego = new Object with behavior Foo([-5, Range(10, 25)])'
    )
    scene = sampleScene(scenario)
    actions1 = sampleEgoActionsFromScene(scene)
    assert 10 <= actions1[0] <= 25
    actions2 = sampleEgoActionsFromScene(scene)
    assert actions1 == actions2
    scene2 = sampleScene(scenario)
    actions3 = sampleEgoActionsFromScene(scene2)
    assert actions1 != actions3

def test_behavior_object_argument():
    scenario = compileScenic("""
        behavior Foo(obj):
            while True:
                take obj.flag
        behavior Bar():
            self.flag = 1
            wait
        other = new Object with flag 0, with behavior Bar
        ego = new Object at (10, 0), with behavior Foo(other)
    """)
    actions = sampleEgoActions(scenario, maxSteps=2)
    assert actions[1] == 1

# Globals

def test_behavior_globals_read():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take other.position.x
        ego = new Object with behavior Foo
        other = new Object at Range(10, 20) @ 15
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
            ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
        foo = [5, Range(10, 20)]
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
        other = new Object with behavior Foo
        ego = new Object at 10@10, with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert len(actions) == 3
    assert actions[0] == True
    assert actions[2] == False

def test_behavior_namespace_interference(runLocally):
    """Test that namespaces of behaviors are isolated acrosss compilations.

    This checks a rather nasty bug wherein under certain complex circumstances
    involving circular imports, a reference to a Scenic module in the global
    namespace of a behavior could leak from one compilation to another.
    """
    with runLocally():
        for i in range(2):
            scenario = compileScenic(f"""
                import submodule.subsub as sub
                sub.myglobal = {i}
                behavior Foo():
                    take sub.subsub.myglobal
                ego = new Object with behavior Foo
            """)
            actions = sampleEgoActions(scenario)
            assert len(actions) == 1
            assert actions[0] == i

# Implicit self

def test_behavior_self():
    scenario = compileScenic("""
        behavior Foo():
            take self.bar
        ego = new Object with behavior Foo, with bar 3
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (3,)

def test_behavior_lazy():
    scenario = compileScenic("""
        vf = VectorField("Foo", lambda pos: pos.x)
        behavior Foo():
            take 1 relative to vf
        ego = new Object at 0.5@0, with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (pytest.approx(1.5),)

def test_behavior_lazy_nested():
    scenario = compileScenic("""
        vf = VectorField("Foo", lambda pos: pos.x)
        behavior Foo():
            do Bar()
            take -1 relative to vf
        behavior Bar():
            take 1 relative to vf
        behavior Baz():
            do Bar(); do Bar()
        new Object at -10@0, with behavior Baz
        ego = new Object at 0.5@0, with behavior Foo
    """)
    actions = sampleActions(scenario, maxSteps=2)
    assert tuple(actions) == (pytest.approx((1.5, -9)), pytest.approx((-0.5, -9)))

# Termination

def test_behavior_end_early():
    scenario = compileScenic("""
        behavior Foo():
            take 5
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (5, None, None)

def test_terminate():
    scenario = compileScenic("""
        behavior Foo():
            take 1
            terminate
            take 2
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
        terminate when flag
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (1, 2)

# Reuse

def test_behavior_reuse():
    scenario = compileScenic(
        'behavior Foo(arg):\n'
        '    take arg\n'
        'ego = new Object with behavior Foo(3)\n'
        'new Object at 10@10, with behavior Foo(5)'
    )
    actions = sampleActions(scenario)
    assert len(actions) == 1
    assert tuple(actions[0]) == (3, 5)

def test_behavior_reuse_2():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    take Range(-10, 10)\n'
        'ego = new Object with behavior Foo\n'
        'new Object at 10@10, with behavior Foo'
    )
    actions = sampleActions(scenario)
    assert len(actions) == 1
    action1, action2 = actions[0]
    assert action1 != action2

# Ordering

def test_behavior_ordering_default():
    scenario = compileScenic("""
        count = 0
        behavior Foo():
            global count
            count += 1
            take count
        new Object with name 'A', with behavior Foo
        new Object with name 'B', at 10@0, with behavior Foo
        ego = new Object with name 'C', at 20@0, with behavior Foo
    """)
    scene = sampleScene(scenario)
    objsByName = {}
    for obj in scene.objects:
        objsByName[obj.name] = obj
    actions = sampleActionsFromScene(scene, asMapping=True)
    assert len(actions) == 1
    actions = actions[0]
    assert actions[objsByName['C']] == 1
    assert actions[objsByName['A']] == 2
    assert actions[objsByName['B']] == 3
    assert tuple(actions.keys()) == scene.objects

# Nesting (sub-behaviors)

def test_behavior_nesting():
    scenario = compileScenic("""
        behavior Foo(a):
            take a
            take a
        behavior Bar():
            take 1
            do Foo(2)
            take 3
        ego = new Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 2, 2, 3)

def test_subbehavior_for_steps():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take 1
        behavior Bar():
            do Foo() for 3 steps
            take 2
        ego = new Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 1, 1, 2)

def test_subbehavior_for_time():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take 1
        behavior Bar():
            do Foo() for 3 seconds
            take 2
        ego = new Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=7, timestep=0.5)
    assert tuple(actions) == (1, 1, 1, 1, 1, 1, 2)

def test_subbehavior_until():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take 1
        behavior Bar():
            do Foo() until simulation().currentTime == 2
            take 2
        ego = new Object with behavior Bar
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 1, 2, None)

def test_behavior_invoke_mistyped():
    scenario = compileScenic("""
        behavior Foo():
            do 12
        ego = new Object with behavior Foo
    """)
    with pytest.raises(RuntimeParseError):
        sampleActions(scenario)

def test_behavior_invoke_multiple():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("""
            behavior Foo():
                take 5
            behavior Bar():
                do Foo(), Foo()
            ego = new Object with behavior Bar
        """)

def test_behavior_calls():
    """Ordinary function calls inside behaviors should still work."""
    scenario = compileScenic("""
        def func(a, *b, c=0, d=1, **e):
            return [a, len(b), c, d, len(e)]
        behavior Foo():
            take [func(4, 5, 6, blah=4, c=10)]
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (1, 2, 3, 4)

# Preconditions and invariants

def test_behavior_precondition():
    scenario = compileScenic("""
        behavior Foo():
            precondition: self.position.x > 0
            take self.position.x
        ego = new Object at Range(-1, 1) @ 0, with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=1, maxIterations=1, maxScenes=50)
        assert actions[0] > 0

def test_behavior_invariant():
    scenario = compileScenic("""
        behavior Foo():
            invariant: self.position.x > 0
            while True:
                take self.position.x
                self.position -= Range(0, 2) @ 0
        ego = new Object at 1 @ 0, with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=3, maxIterations=50)
        assert actions[1] > 0

# Random selection of sub-behaviors

def test_choose_1():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                do choose Bar(1), Bar(2)
        behavior Bar(x):
            take x
        ego = Object with behavior Foo
    """)
    ts = [sampleEgoActions(scenario, maxSteps=2) for i in range(40)]
    assert any(t[0] == 1 for t in ts)
    assert any(t[0] == 2 for t in ts)
    assert any(t[0] == t[1] for t in ts)
    assert any(t[0] != t[1] for t in ts)

def test_choose_2():
    scenario = compileScenic("""
        behavior Foo():
            do choose Bar(1), Bar(2)
            terminate
        behavior Bar(p):
            precondition: self.position.x == p
            take (self.position.x == p)
        ego = Object at Uniform(1, 2) @ 0, with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=2)
        assert len(actions) == 1
        assert actions[0] == True

def test_choose_3():
    scenario = compileScenic("""
        behavior Foo():
            do choose {Sub(0): 1, Sub(1): 9}
        behavior Sub(x):
            take x
        ego = Object with behavior Foo
    """)
    xs = [sampleEgoActions(scenario)[0] for i in range(200)]
    assert all(x == 0 or x == 1 for x in xs)
    assert 145 <= sum(xs) < 200

def test_choose_deadlock():
    scenario = compileScenic("""
        behavior Foo():
            do choose Bar(1), Bar(2)
        behavior Bar(p):
            precondition: self.position.x == p
            wait
        ego = Object at 3 @ 0, with behavior Foo
    """)
    result = sampleResultOnce(scenario)
    assert result is None

def test_shuffle_1():
    scenario = compileScenic("""
        behavior Foo():
            do shuffle Sub(-1), Sub(1)
            terminate
        behavior Sub(x):
            precondition: simulation().currentTime >= x
            take x
        ego = Object with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=3)
        assert tuple(actions) == (-1, 1)

def test_shuffle_2():
    scenario = compileScenic("""
        behavior Foo():
            do shuffle Sub(1), Sub(3)
        behavior Sub(x):
            take x
        ego = Object with behavior Foo
    """)
    ts = [sampleEgoActions(scenario, maxSteps=2) for i in range(30)]
    assert all(tuple(t) == (1, 3) or tuple(t) == (3, 1) for t in ts)
    assert any(tuple(t) == (1, 3) for t in ts)
    assert any(tuple(t) == (3, 1) for t in ts)

def test_shuffle_3():
    scenario = compileScenic("""
        behavior Foo():
            do shuffle {Sub(0): 1, Sub(1): 9}
        behavior Sub(x):
            take x
        ego = Object with behavior Foo
    """)
    ts = [sampleEgoActions(scenario, maxSteps=2) for i in range(200)]
    assert all(tuple(t) == (0, 1) or tuple(t) == (1, 0) for t in ts)
    assert 145 <= sum(t[0] for t in ts) < 200

def test_shuffle_deadlock():
    scenario = compileScenic("""
        behavior Foo():
            do shuffle Sub(), Sub()
        behavior Sub():
            precondition: simulation().currentTime == 0
            wait
        ego = Object with behavior Foo
    """)
    result = sampleResultOnce(scenario, maxSteps=2)
    assert result is None

# Requirements

def test_behavior_require():
    scenario = compileScenic("""
        behavior Foo():
            x = Range(-1, 1)
            while True:
                take x
                require x < 0
        ego = new Object with behavior Foo
    """)
    for i in range(50):
        actions = sampleEgoActions(scenario, maxSteps=2, maxIterations=50, maxScenes=1)
        assert len(actions) == 2
        assert actions[0] < 0
        assert actions[0] == actions[1]

def test_behavior_require_scene():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take self.foo
                require self.foo < 0
        ego = new Object with foo Range(-1, 1), with behavior Foo
    """)
    for i in range(50):
        actions = sampleEgoActions(scenario, maxSteps=2, maxIterations=1, maxScenes=50)
        assert len(actions) == 2
        assert actions[0] < 0
        assert actions[0] == actions[1]

def test_behavior_require_call():
    scenario = compileScenic("""
        behavior Foo():
            x = Uniform([], [1, 2])
            require len(x) > 0
            take [x]
        ego = new Object with behavior Foo
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=1, maxIterations=30)
        assert actions[0] == [1, 2]

## Temporal requirements

def test_require_always():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take self.blah
                self.blah += DiscreteRange(0, 1)
        ego = new Object with behavior Foo, with blah 0
        require always ego.blah < 1
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=2, maxIterations=50)
        assert tuple(actions) == (0, 0)

def test_require_eventually():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take self.blah
                self.blah += DiscreteRange(0, 1)
        ego = new Object with behavior Foo, with blah 0
        require eventually ego.blah > 0
    """)
    for i in range(30):
        actions = sampleEgoActions(scenario, maxSteps=2, maxIterations=50)
        assert tuple(actions) == (0, 1)

def test_require_eventually_2():
    scenario = compileScenic("""
        behavior Foo():
            while True:
                take self.blah
                self.blah += 1
        ego = new Object with behavior Foo, with blah 0
        require eventually ego.blah == 0
        require eventually ego.blah == 1
        require eventually ego.blah == 2
    """)
    sampleEgoActions(scenario, maxSteps=3)

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
        ego = new Object with blah 0, with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (0, 1, 2, 3)

## Interrupts

# Basic

def test_interrupt():
    scenario = compileScenic("""
        behavior Foo():
            try:
                while True:
                    take 1
            interrupt when simulation().currentTime % 3 == 2:
                take 2
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=6)
    assert tuple(actions) == (1, 1, 2, 1, 1, 2)

def test_interrupt_first():
    scenario = compileScenic("""
        behavior Foo():
            try:
                while True:
                    take 1
            interrupt when simulation().currentTime == 0:
                take 2
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (2, 1, 1)

def test_interrupt_priority():
    scenario = compileScenic("""
        behavior Foo():
            try:
                while True:
                    take 1
            interrupt when simulation().currentTime <= 1:
                take 2
            interrupt when simulation().currentTime == 0:
                take 3
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (3, 2, 1)

def test_interrupt_interrupted():
    scenario = compileScenic("""
        behavior Foo():
            try:
                while True:
                    take 1
            interrupt when simulation().currentTime <= 1:
                take 2
                take 3
            interrupt when simulation().currentTime == 1:
                take 4
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (2, 4, 3, 1, 1)

def test_interrupt_actionless():
    scenario = compileScenic("""
        behavior Foo():
            i = 0
            try:
                for i in range(3):
                    take 1
            interrupt when i == 1:
                i = 2
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=5)
    assert tuple(actions) == (1, 1, 1, None, None)

def test_interrupt_define_local():
    scenario = compileScenic("""
        behavior Foo():
            try:
                i = 1
            interrupt when False:
                pass
            take i
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (1,)

def test_interrupt_define_local_2():
    scenario = compileScenic("""
        behavior Foo():
            try:
                pass
            interrupt when True:
                i = 1
                abort
            take i
        ego = Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=1)
    assert tuple(actions) == (1,)

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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
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
                do Foo()
            interrupt when simulation().currentTime == 1:
                take 3
        ego = new Object with behavior Bar
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
                do Foo()
            interrupt when simulation().currentTime == 2:
                take 4
        ego = new Object with behavior Bar
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
        ego = new Object with behavior Bar
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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
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
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=8)
    assert tuple(actions) == (3, 1, 2, 3, 1, 1, 1, 3)

def test_interrupt_return():
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
                        return
        ego = new Object with behavior Foo
    """)
    actions = sampleEgoActions(scenario, maxSteps=4)
    assert tuple(actions) == (3, 1, 2, None)

# Errors

def test_interrupt_unassigned_local():
    scenario = compileScenic("""
        behavior Foo():
            try:
                i = 0
                take 1
            interrupt when i == 1:
                i = 2
        ego = new Object with behavior Foo
    """)
    if sys.version_info >= (3, 10, 3):  # see veneer.executeInBehavior
        exc_type = NameError
    else:
        exc_type = AttributeError
    with pytest.raises(exc_type) as exc_info:
        sampleEgoActions(scenario, maxSteps=1)
    checkErrorLineNumber(5, exc_info)

def test_interrupt_guard_subbehavior():
    scenario = compileScenic("""
        behavior Foo():
            try:
                take 1
            interrupt when Foo():
                wait
        ego = new Object with behavior Foo
    """)
    with pytest.raises(RuntimeParseError):
        sampleEgoActions(scenario, maxSteps=1)

## Simulation results

def test_termination_reason_time():
    scenario = compileScenic("""
        ego = new Object
    """)
    result = sampleResult(scenario, maxSteps=2)
    assert result.terminationType == TerminationType.timeLimit

def test_termination_reason_condition_1():
    scenario = compileScenic("""
        behavior Foo():
            for i in range(3):
                self.position = self.position + 1@0
                wait
        ego = new Object with behavior Foo
        terminate when ego.position.x >= 1
    """)
    result = sampleResult(scenario, maxSteps=2)
    assert result.terminationType == TerminationType.scenarioComplete

def test_termination_reason_condition_2():
    scenario = compileScenic("""
        behavior Foo():
            for i in range(3):
                self.position = self.position + 1@0
                wait
        ego = new Object with behavior Foo
        terminate simulation when ego.position.x >= 1
    """)
    result = sampleResult(scenario, maxSteps=2)
    assert result.terminationType == TerminationType.simulationTerminationCondition

def test_termination_reason_behavior():
    scenario = compileScenic("""
        behavior Foo():
            terminate
        ego = new Object with behavior Foo
    """)
    result = sampleResult(scenario, maxSteps=2)
    assert result.terminationType == TerminationType.terminatedByBehavior

def test_termination_reason_monitor():
    scenario = compileScenic("""
        monitor Foo:
            terminate
        ego = new Object
    """)
    result = sampleResult(scenario, maxSteps=2)
    assert result.terminationType == TerminationType.terminatedByMonitor

## Recording

def test_record():
    scenario = compileScenic("""
        behavior Foo():
            for i in range(3):
                self.position = self.position + 2@0
                wait
        ego = new Object with behavior Foo
        terminate when ego.position.x >= 6
        record initial ego.position as initial
        record final ego.position as final
        record ego.position as position
    """)
    result = sampleResult(scenario, maxSteps=4)
    assert result.records['initial'] == (0, 0)
    assert result.records['final'] == (6, 0)
    assert tuple(result.records['position']) == (
        (0, (0, 0)),
        (1, (2, 0)),
        (2, (4, 0)),
        (3, (6, 0))
    )
