
import pytest

from scenic.core.distributions import Range

from tests.utils import compileScenic

def test_nonexistent_scenario_local_1():
    scenario = compileScenic('ego = Object')
    with pytest.raises(AttributeError):
        scenario.dynamicScenario.blah

def test_nonexistent_scenario_local_2():
    scenario = compileScenic("""
        scenario Main():
            ego = Object
    """)
    with pytest.raises(AttributeError):
        scenario.dynamicScenario.blah

def test_condition_scenario_objects():
    scenario = compileScenic("""
        Object facing Range(0, 1)
        ego = Object at 10@10, facing Range(0, 1)
    """)
    sceneA, _ = scenario.generate(maxIterations=1)
    scenario.conditionOn(scene=sceneA, objects=(1,))
    sceneB, _ = scenario.generate(maxIterations=1)
    assert sceneA.objects[0].heading != sceneB.objects[0].heading
    assert sceneA.objects[1].heading == sceneB.objects[1].heading

def test_condition_scenario_params_1():
    scenario = compileScenic("""
        ego = Object
        param x = Range(0, 1)
    """)
    scenario.conditionOn(params={'x': 0.6})
    scene, _ = scenario.generate(maxIterations=1)
    assert scene.params['x'] == 0.6

def test_condition_scenario_params_2():
    scenario = compileScenic("""
        ego = Object
        param x = Range(0, 1)
    """)
    scenario.conditionOn(params={'x': Range(0.5, 0.51)})
    xs = [scenario.generate(maxIterations=1)[0].params['x'] for i in range(30)]
    assert all(0.5 <= x <= 0.51 for x in xs)
    assert any(0.505 <= x for x in xs)
    assert any(x < 0.505 for x in xs)
