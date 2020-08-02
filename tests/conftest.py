
import os.path
from contextlib import contextmanager
import pytest

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

## Command-line option to skip very slow tests

def pytest_addoption(parser):
    parser.addoption('--fast', action='store_true', help='skip very slow tests')

def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: mark test as very slow')

def pytest_collection_modifyitems(config, items):
    if config.getoption('--fast'):
        mark = pytest.mark.skip(reason='slow test skipped by --fast')
        for item in items:
            if 'slow' in item.keywords:
                item.add_marker(mark)
