"""Tests for various ways to serialize Scenic scenes/scenarios.

For pickling, see ``test_pickle.py`` and many other tests marked with
`pickle_test` throughout the test suite.
"""

import io

from tests.utils import sampleSceneFrom

# Exporting scenes to Scenic code

class TestExportToScenicCode:
    def test_simple(self):
        scene1 = sampleSceneFrom("""
            ego = Object at Range(3, 5) @ 2, with foo 42
            Object at 10@10, facing toward ego, with foo 'zoggle'
            param qux = ego.position
        """)
        stream = io.StringIO()
        scene1.dumpAsScenicCode(stream)
        code = stream.getvalue()
        scene2 = sampleSceneFrom(code)
        assert len(scene1.objects) == len(scene2.objects) == 2
        assert 3 <= scene1.egoObject.position.x <= 5
        for obj1, obj2 in zip(scene1.objects, scene2.objects):
            assert obj1.position == obj2.position
            assert obj1.heading == obj2.heading
            assert obj1.foo == obj2.foo
            assert obj1.width == obj2.width
        assert len(scene1.params) == len(scene2.params) == 1
        assert scene1.params['qux'] == scene2.params['qux']
