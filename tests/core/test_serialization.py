"""Tests for various ways to serialize Scenic scenes/scenarios.

For pickling, see ``test_pickle.py`` and many other tests marked with
`pickle_test` throughout the test suite.
"""

import io
import random
import subprocess
import sys

import pytest

from tests.utils import compileScenic, sampleScene, sampleSceneFrom

simpleScenario = """
    ego = Object at Range(3, 5) @ 2, with foo Uniform('zoggle', 'buggle')
    Object at 10@10, facing toward ego, with foo 42
    param qux = ego.position
"""
def checkSimpleScenes(scene1, scene2):
    assert len(scene1.objects) == len(scene2.objects) == 2
    assert 3 <= scene1.egoObject.position.x <= 5
    for obj1, obj2 in zip(scene1.objects, scene2.objects):
        assert obj1.position == obj2.position
        assert obj1.heading == obj2.heading
        assert obj1.foo == obj2.foo
        assert obj1.width == obj2.width
    assert len(scene1.params) == len(scene2.params) == 1
    assert scene1.params['qux'] == scene2.params['qux']

# Exporting scenes to Scenic code

class TestExportToScenicCode:
    def test_simple(self):
        scene1 = sampleSceneFrom(simpleScenario)
        stream = io.StringIO()
        scene1.dumpAsScenicCode(stream)
        code = stream.getvalue()
        scene2 = sampleSceneFrom(code)
        checkSimpleScenes(scene1, scene2)

# Serializing scenes given a compiled scenario

def checkHelper(seed, data):
    import scenic.core.errors as errors
    errors.showInternalBacktrace = True
    scenario = compileScenic(simpleScenario)
    scene1 = scenario.sceneFromBytes(data)
    random.seed(seed)
    scene2 = sampleScene(scenario)
    checkSimpleScenes(scene1, scene2)

class TestExportToBytes:
    def test_simple(self):
        scenario = compileScenic(simpleScenario)
        scene1 = sampleScene(scenario)
        data = scenario.sceneToBytes(scene1)
        scene2 = scenario.sceneFromBytes(data)
        checkSimpleScenes(scene1, scene2)

    @pytest.mark.slow
    def test_separate_interpreters(self):
        for i in range(20):
            seed = random.randint(0, 1000000)
            scenario = compileScenic(simpleScenario)
            random.seed(seed)
            scene1 = sampleScene(scenario)
            data = scenario.sceneToBytes(scene1)
            command = (
                'from tests.core.test_serialization import checkHelper;'
                f'checkHelper({seed}, {data})'
            )
            args = [sys.executable, '-c', command]
            result = subprocess.run(args, capture_output=True, text=True)
            assert result.returncode == 0, result.stderr
