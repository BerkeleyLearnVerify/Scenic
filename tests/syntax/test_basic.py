
import pytest

import scenic
from scenic import scenarioFromString as compileScenic
from scenic.syntax.translator import InvalidScenarioError, InterpreterParseError
from scenic.core.object_types import Object

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
    scene, iterations = scenario.generate(maxIterations=1)
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
    scene, iterations = scenario.generate(maxIterations=1)
    assert len(scene.objects) == 2
    obj = scene.objects[0]
    assert obj is scene.egoObject

def test_ego_nonobject():
    with pytest.raises(InterpreterParseError):
        compileScenic('ego = Point')
    with pytest.raises(InterpreterParseError):
        compileScenic('ego = dict()')

def test_ego_undefined():
    with pytest.raises(InterpreterParseError):
        compileScenic('x = ego\n' 'ego = Object')

def test_noninterference():
    scenario = compileScenic('ego = Object')
    assert len(scenario.objects) == 1
    ego1 = scenario.egoObject
    for i in range(5):
        scene, iterations = scenario.generate(maxIterations=1)
    scenario = compileScenic('ego = Object')
    assert len(scenario.objects) == 1
    ego2 = scenario.egoObject
    assert ego1 is not ego2

def test_verbose():
    for verb in range(4):
        scenic.syntax.translator.verbosity = verb
        compileScenic('ego = Object')
    scenic.syntax.translator.verbosity = 1

def test_dump_python():
    scenic.syntax.translator.dumpTranslatedPython = True
    compileScenic('ego = Object')
    scenic.syntax.translator.dumpTranslatedPython = False
    scenic.syntax.translator.dumpFinalAST = True
    compileScenic('ego = Object')
    scenic.syntax.translator.dumpFinalAST = False
