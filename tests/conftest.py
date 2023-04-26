
from contextlib import contextmanager
import os.path
import subprocess
from pathlib import Path
import re
import sys

import pytest

from scenic.syntax import buildParser

## Fixtures for use in tests

@pytest.fixture
def loadLocalScenario(request):
    import scenic
    base = os.path.dirname(request.fspath)
    def loader(relpath):
        path = os.path.join(base, relpath)
        return scenic.scenarioFromFile(path)
    return loader

@pytest.fixture
def loadLocalScenario2D(request):
    import scenic
    base = os.path.dirname(request.fspath)
    def loader(relpath):
        path = os.path.join(base, relpath)
        return scenic.scenarioFromFile(path, mode_2d=True)
    return loader

@pytest.fixture
def runLocally(request):
    base = os.path.dirname(request.fspath)
    @contextmanager
    def manager():
        oldDirectory = os.getcwd()
        os.chdir(base)
        try:
            yield
        finally:
            os.chdir(oldDirectory)
    return manager

## Command-line options

def pytest_addoption(parser):
    # option to skip very slow tests
    parser.addoption('--fast', action='store_true', help='skip very slow tests')
    # option to skip graphical tests
    parser.addoption('--no-graphics', action='store_true', help='skip graphical tests')
    # option to skip parser generation
    parser.addoption('--skip-pegen', action='store_true', help='skip generating the parser before running tests')

def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: mark test as very slow')
    config.addinivalue_line('markers', 'graphical: mark test as requiring graphics')

    if not config.getoption("skip_pegen"):
        result = buildParser()
        if result.returncode != 0:
            pytest.exit(f"Failed to generate the parser: {result.stderr}", result.returncode)

def pytest_collection_modifyitems(config, items):
    if config.getoption('--fast'):
        mark = pytest.mark.skip(reason='slow test skipped by --fast')
        for item in items:
            if 'slow' in item.keywords:
                item.add_marker(mark)
    if config.getoption('--no-graphics'):
        mark = pytest.mark.skip(reason='graphical test skipped by --no-graphics')
        for item in items:
            if 'graphical' in item.keywords:
                item.add_marker(mark)

def pytest_ignore_collect(path, config):
    """
    Some test files use Python syntax (e.g., structural pattern matching) that is only available in newer versions of Python.
    To avoid syntax errors on older versions, put `# pytest: python>=x.y` where x and y represent major and minor versions, respectively,
    required to run the test file.
    """
    if path.isfile() and path.ext in ('.py', '.scenic'):
        firstLine = ""
        with path.open() as f:
            firstLine = f.readline()
        m = re.search(r"^#\s*pytest:\s*python\s*>=\s*(?P<major>\d+)\.(?P<minor>\d+)\s*$", firstLine)
        if m:
            target = (int(m.group("major")), int(m.group("minor")))
            return True if sys.version_info < target else None
    return None
