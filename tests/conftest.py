
import os.path
import subprocess
from pathlib import Path
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


def pytest_addoption(parser):
    # option to skip very slow tests
    parser.addoption('--fast', action='store_true', help='skip very slow tests')
    # option to skip parser generation
    parser.addoption('--skip-pegen', action='store_true', help='skip generating the parser before running tests')

class ParserGenerationException(BaseException):
    pass

def pytest_configure(config):
    config.addinivalue_line('markers', 'slow: mark test as very slow')

    if not config.getoption("skip_pegen"):
        projectRootDir = Path(__file__).parent.parent
        syntaxDir = projectRootDir / "src" / "scenic" / "syntax"
        grammar = syntaxDir / "scenic.gram"
        parser = syntaxDir / "parser.py"
        result = subprocess.run(
            [
                "python",
                "-m",
                "pegen",
                grammar,
                "-o",
                parser,
            ],
            cwd=projectRootDir,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise ParserGenerationException(f"Failed to generate the parser: {result.stderr}")

def pytest_collection_modifyitems(config, items):
    if config.getoption('--fast'):
        mark = pytest.mark.skip(reason='slow test skipped by --fast')
        for item in items:
            if 'slow' in item.keywords:
                item.add_marker(mark)
