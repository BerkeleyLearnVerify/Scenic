
from tests.utils import (compileScenic, sampleScene, sampleActions, sampleEgoActions,
                         sampleEgoActionsFromScene)

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

def test_behavior_end_early():
    scenario = compileScenic(
        'behavior Foo():\n'
        '    take 5\n'
        'ego = Object with behavior Foo'
    )
    actions = sampleEgoActions(scenario, maxSteps=3)
    assert tuple(actions) == (5, None, None)

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
