"""Tests for various ways to serialize Scenic scenes/scenarios.

For pickling, see ``test_pickle.py`` and many other tests marked with
`pickle_test` throughout the test suite.
"""

import io
import random
import subprocess
import sys

import pytest

from scenic.core.serialization import Serializer
from tests.utils import (areEquivalent, compileScenic, sampleScene, sampleSceneFrom,
                         sampleEgoActionsFromScene)

simpleScenario = """
    ego = Object at Range(3, 5) @ 2, with foo Uniform('zoggle', 'buggle')
    Object at 10@10, facing toward ego, with foo Options({Range(1,2): 1, Range(3,4): 2})
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

def subprocessHelper(seed, data):
    import scenic.core.errors as errors
    errors.showInternalBacktrace = True
    scenario = compileScenic(simpleScenario)
    scene1 = scenario.sceneFromBytes(data)
    assert scenario.sceneToBytes(scene1) == data
    random.seed(seed)
    scene2 = sampleScene(scenario)
    checkSimpleScenes(scene1, scene2)
    assert scenario.sceneToBytes(scene2) == data
    # Samples may not be equivalent since we only serialize random values which actually
    # get used in the scene; so need to delete them before checking equivalence.
    del scene1.sample, scene2.sample
    assert areEquivalent(scene1, scene2)

def checkValueEncoding(val, ty, allowPickle=False):
    enc = Serializer(allowPickle=allowPickle)
    enc.writeValue(val, ty)
    data = enc.getBytes()
    dec = Serializer(data, allowPickle=allowPickle)
    val2 = dec.readValue(ty)
    assert areEquivalent(val, val2)

class Thingy:
    def __init__(self, x):
        self.x = x

class TestExportToBytes:
    def test_int(self):
        checkValueEncoding(0, int)
        checkValueEncoding(42, int)
        checkValueEncoding(-42, int)
        checkValueEncoding(256, int)
        checkValueEncoding(-1724, int)
        checkValueEncoding(123456, int)
        checkValueEncoding(-123456, int)
        checkValueEncoding(3**30, int)
        checkValueEncoding(-5**21, int)

    def test_float(self):
        checkValueEncoding(3.14159, float)
        checkValueEncoding(-2.71828, float)
        checkValueEncoding(4.123e50, float)
        checkValueEncoding(7.89e-50, float)
        checkValueEncoding(float('inf'), float)
        checkValueEncoding(float('nan'), float)

    def test_object_with_encodeTo(self):
        from scenic.simulators.utils.colors import Color
        checkValueEncoding(Color(0.5, 1.0, 0.2), Color)
        from scenic.core.vectors import Vector
        checkValueEncoding(Vector(-7.5, 42), Vector)

        class Foo:
            def __init__(self, x):
                self.x = x

            @staticmethod
            def encodeTo(value, stream):
                stream.write(bytes([1, 2, value.x, 4]))

            @staticmethod
            def decodeFrom(stream):
                data = stream.read(4)
                return Foo(data[2])
        checkValueEncoding(Foo(13), Foo)

    def test_object_pickle(self):
        checkValueEncoding(Thingy(42), object, allowPickle=True)
        checkValueEncoding(Thingy('foo'), Thingy, allowPickle=True)

    def test_simple_scene(self):
        scenario = compileScenic(simpleScenario)
        scene1 = sampleScene(scenario)
        data = scenario.sceneToBytes(scene1)
        scene2 = scenario.sceneFromBytes(data)
        checkSimpleScenes(scene1, scene2)
        assert scenario.sceneToBytes(scene2) == data

    def test_scene_behavior(self):
        scenario = compileScenic("""
            glob = Range(1, 2)
            behavior Foo(x):
                take glob
                take x
            ego = Object with behavior Foo(Range(3, 4))
        """)
        scene1 = sampleScene(scenario)
        a11, a12 = sampleEgoActionsFromScene(scene1, maxSteps=2)
        assert 1 <= a11 <= 2
        assert 3 <= a12 <= 4
        data = scenario.sceneToBytes(scene1)
        scene2 = scenario.sceneFromBytes(data)
        a21, a22 = sampleEgoActionsFromScene(scene2, maxSteps=2)
        assert a21 == a11
        assert a22 == a12
        print(a11,a21,a12,a22)

    @pytest.mark.slow
    def test_separate_interpreters(self):
        # Repeat this test several times to catch nondeterminism due to hash
        # randomization, for example
        for i in range(20):
            seed = random.randint(0, 1000000)
            scenario = compileScenic(simpleScenario)
            random.seed(seed)
            scene1 = sampleScene(scenario)
            data = scenario.sceneToBytes(scene1)
            command = (
                'from tests.core.test_serialization import subprocessHelper;'
                f'subprocessHelper({seed}, {data})'
            )
            args = [sys.executable, '-c', command]
            result = subprocess.run(args, capture_output=True, text=True)
            assert result.returncode == 0, result.stderr
