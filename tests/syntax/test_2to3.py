
import inspect
import io
import os

import pytest

from scenic.syntax.scenic2to3 import Scenic2to3

@pytest.fixture
def check(tmpdir):
    inPath = os.path.join(tmpdir, 'in.scenic')
    outPath = os.path.join(tmpdir, 'out.scenic')
    def checker(code, expected, params={}, model=None):
        code = inspect.cleandoc(code)
        expected = inspect.cleandoc(expected)
        with open(inPath, 'w') as inFile:
            inFile.write(code)
        converter = Scenic2to3(inPath, params, model)
        converter.dump(outPath)
        with open(outPath, 'r') as outFile:
            result = outFile.read()
        assert result == expected
    return checker

@pytest.mark.parametrize('prefix', ('', 'ego = ', 'blob='))
@pytest.mark.parametrize('specifiers', ('', ' with foo 42'))
@pytest.mark.parametrize('suffix', ('', '\n', '  # foo'))
def test_object_creation(check, prefix, specifiers, suffix):
    check(f'{prefix}Object{specifiers}{suffix}',
          f'{prefix}new Object{specifiers}{suffix}')

def test_custom_object_creation(check):
    check("""
        class Blob:
            pass
        Blob
        """,
        """
        class Blob:
            pass
        new Blob
        """)

@pytest.mark.parametrize('code', ('[Point, OrientedPoint, Object]', 'Exception'))
def test_class_noncreation(check, code):
    check(code, code)

def test_monitor(check):
    check("""
        monitor MyMon:
            while True:
                require ego.foo > distance to taxi
                wait
        """,
        """
        monitor MyMon():
            while True:
                require ego.foo > distance to taxi
                wait
        require monitor MyMon()
        """)

def test_param(check):
    code = 'globalParameters.foo'
    with pytest.raises(KeyError):   # sanity check
        check(code, code)
    check(code, code, params={'foo': 42})
