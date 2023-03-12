
import matplotlib.pyplot as plt
import pytest

import scenic
from scenic.core.errors import (InvalidScenarioError, ScenicSyntaxError,
                                setDebuggingOptions)
from scenic.core.object_types import Object
from tests.utils import compileScenic, sampleScene, sampleEgo, sampleParamPFrom

def test_empty():
    with pytest.raises(InvalidScenarioError):
        compileScenic('')

def test_minimal():
    scenario = compileScenic('ego = Object')
    assert len(scenario.objects) == 1
    obj = scenario.objects[0]
    assert type(obj) is Object
    assert obj is scenario.egoObject
    assert len(scenario.params) == 0
    assert len(scenario.requirements) == 0
    scene = sampleScene(scenario, maxIterations=1)
    assert len(scene.objects) == 1
    obj = scene.objects[0]
    assert type(obj) is Object
    assert obj is scene.egoObject
    assert len(scene.params) == 0

def test_ego_second():
    scenario = compileScenic('Object\n' 'ego = Object at 0 @ -5')
    assert len(scenario.objects) == 2
    obj = scenario.objects[0]
    assert obj is scenario.egoObject
    scene = sampleScene(scenario, maxIterations=1)
    assert len(scene.objects) == 2
    obj = scene.objects[0]
    assert obj is scene.egoObject

def test_ego_nonobject():
    with pytest.raises(ScenicSyntaxError):
        compileScenic('ego = Point')
    with pytest.raises(ScenicSyntaxError):
        compileScenic('ego = dict()')

def test_ego_undefined():
    with pytest.raises(ScenicSyntaxError):
        compileScenic('x = ego\n' 'ego = Object')

def test_ego_complex_assignment():
    with pytest.raises(ScenicSyntaxError):
        compileScenic('(ego, thing1), thing2 = ((Object at 1@1), 2), 3')

def test_noninterference():
    scenario = compileScenic('ego = Object')
    assert len(scenario.objects) == 1
    ego1 = scenario.egoObject
    for i in range(5):
        scene = sampleScene(scenario, maxIterations=1)
    scenario = compileScenic('ego = Object')
    assert len(scenario.objects) == 1
    ego2 = scenario.egoObject
    assert ego1 is not ego2

def test_param():
    p = sampleParamPFrom('ego = Object\n' 'param p = Range(3, 5)')
    assert 3 <= p <= 5
    p = sampleParamPFrom('ego = Object\n' 'param p = [1, 4, 9]')
    assert type(p) is list
    assert p == [1, 4, 9]
    p = sampleParamPFrom('ego = Object\n' 'param p = (1, 4)')
    assert type(p) is tuple
    assert p == (1, 4)

def test_quoted_param():
    p = sampleParamPFrom('ego = Object\n' 'param "p" = Range(3, 5)')
    assert 3 <= p <= 5

def test_param_read():
    p = sampleParamPFrom("""
        ego = Object
        param q = Range(3, 5)
        param p = globalParameters.q + 10
    """)
    assert 13 <= p <= 15

def test_param_write():
    with pytest.raises(ScenicSyntaxError):
        compileScenic('ego = Object\n' 'globalParameters = {}')

def test_mutate():
    scenario = compileScenic("""
        ego = Object at 3@1, facing 0
        mutate
    """)
    ego1 = sampleEgo(scenario)
    assert ego1.position.x != pytest.approx(3)
    assert ego1.position.y != pytest.approx(1)
    assert ego1.heading != pytest.approx(0)

def test_mutate_object():
    scenario = compileScenic("""
        ego = Object at 30@1, facing 0
        other = Object
        mutate other
    """)
    scene = sampleScene(scenario)
    ego, other = scene.objects
    assert ego.position.x == pytest.approx(30)
    assert ego.position.y == pytest.approx(1)
    assert ego.heading == pytest.approx(0)
    assert other.position.x != pytest.approx(0)
    assert other.position.y != pytest.approx(0)
    assert other.heading != pytest.approx(0)

def test_mutate_scaled():
    scenario = compileScenic("""
        ego = Object at 3@1, facing 0
        mutate ego by 4
    """)
    ego1 = sampleEgo(scenario)
    assert ego1.position.x != pytest.approx(3)
    assert ego1.position.y != pytest.approx(1)
    assert ego1.heading != pytest.approx(0)

def test_mutate_nonobject():
    with pytest.raises(ScenicSyntaxError):
        compileScenic("""
            ego = Object
            mutate sin
        """)

def test_verbose():
    for verb in range(4):
        setDebuggingOptions(verbosity=verb)
        compileScenic('ego = Object')
    setDebuggingOptions(verbosity=1)

def test_dump_python():
    scenic.syntax.translator.dumpTranslatedPython = True
    try:
        compileScenic('ego = Object')
    finally:
        scenic.syntax.translator.dumpTranslatedPython = False
    scenic.syntax.translator.dumpFinalAST = True
    try:
        compileScenic('ego = Object')
    finally:
        scenic.syntax.translator.dumpFinalAST = False

def test_dump_final_python():
    pytest.importorskip('astor')
    scenic.syntax.translator.dumpASTPython = True
    try:
        compileScenic('ego = Object')
    finally:
        scenic.syntax.translator.dumpASTPython = False

@pytest.mark.graphical
def test_show():
    scenario = compileScenic('ego = Object with color (0.5, 1.0, 0.5)')
    scene = sampleScene(scenario)
    scene.show(block=False)
    plt.close()

@pytest.mark.graphical
def test_show_zoom():
    scenario = compileScenic("""
        ego = Object
        Object at 10@20
    """)
    scene = sampleScene(scenario)
    scene.show(zoom=1, block=False)
    xmin, xmax, ymin, ymax = plt.axis()
    assert xmin < 0 and xmax > 10
    assert ymin < 0 and ymax > 20
    plt.close()
