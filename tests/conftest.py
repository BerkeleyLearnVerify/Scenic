
import os.path
from contextlib import contextmanager
import pytest

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
