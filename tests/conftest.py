from contextlib import contextmanager
import os.path
from pathlib import Path
import re
import sys
import subprocess
import pytest
import time
from scenic.syntax import buildParser

## Fixtures for use in tests


@pytest.fixture
def loadLocalScenario(request):
    import scenic

    base = os.path.dirname(request.fspath)

    def loader(relpath, **kwargs):
        path = os.path.join(base, relpath)
        return scenic.scenarioFromFile(path, **kwargs)

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


@pytest.fixture
def getAssetPath():
    base = Path(__file__).parent.parent / "assets"

    def loader(relpath, **kwargs):
        path = os.path.join(base, relpath)
        return path

    return loader

def clean_up():
    try:
        port = 2000
        pid = subprocess.check_output(["lsof", "-ti", f":{port}", "-sTCP:LISTEN"], text=True).strip()
        print(f"PID: {pid}")
        if len(pid) != 0:
            try:
                subprocess.check_call(["kill", "-9", pid])
            except subprocess.CalledProcessError:
                pass
            time.sleep(2)
    except:
        pass

@pytest.fixture
def launchCarlaServer():
    # clean_up()
    carla_process = subprocess.Popen("bash /software/CARLA_0.9.14/CarlaUE4.sh -RenderOffScreen", shell=True)
    time.sleep(3)
    yield
    while carla_process.poll() is None:
        carla_process.kill()
        time.sleep(0.1)
    # NOTE: CARLA server hangs. The first `kill` usually works, 
    # but for safe measure we needed to kill hanging zombie processes
    for _ in range(10):
        carla_process.kill()
        time.sleep(0.1)

@pytest.fixture
def getCarlaSimulator():
    from scenic.simulators.carla import CarlaSimulator
    base = Path(__file__).parent.parent / "assets" / "maps" / "CARLA"

    def _getCarlaSimulator(town):
            path = os.path.join(base, town + '.xodr')
            simulator = CarlaSimulator(carla_map=town, map_path=path)
            return (simulator, town, path)
    
    return _getCarlaSimulator

## Command-line options


def pytest_addoption(parser):
    # option to skip very slow tests
    parser.addoption("--fast", action="store_true", help="skip very slow tests")
    # option to run tests that exhaustively test certain elements, but can be VERY slow.
    parser.addoption(
        "--exhaustive",
        action="store_true",
        help="run exhaustive tests (too slow to run by default)",
    )
    # option to skip graphical tests
    parser.addoption("--no-graphics", action="store_true", help="skip graphical tests")
    # option to skip parser generation
    parser.addoption(
        "--skip-pegen",
        action="store_true",
        help="skip generating the parser before running tests",
    )


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: mark test as very slow")
    config.addinivalue_line(
        "markers", "exhaustive: mark test as extremely slow and not run by default"
    )
    config.addinivalue_line("markers", "graphical: mark test as requiring graphics")

    if not config.getoption("skip_pegen"):
        result = buildParser()
        if result.returncode != 0:
            pytest.exit(
                f"Failed to generate the parser: {result.stderr}", result.returncode
            )


def pytest_collection_modifyitems(config, items):
    if config.getoption("--fast"):
        mark = pytest.mark.skip(reason="slow test skipped by --fast")
        for item in items:
            if "slow" in item.keywords:
                item.add_marker(mark)
    if not config.getoption("--exhaustive"):
        mark = pytest.mark.skip(
            reason="exhaustive test not run because --exhaustive was not set"
        )
        for item in items:
            if "exhaustive" in item.keywords:
                item.add_marker(mark)
    if config.getoption("--no-graphics"):
        mark = pytest.mark.skip(reason="graphical test skipped by --no-graphics")
        for item in items:
            if "graphical" in item.keywords:
                item.add_marker(mark)


def pytest_ignore_collect(path, config):
    """
    Some test files use Python syntax (e.g., structural pattern matching) that is only available in newer versions of Python.
    To avoid syntax errors on older versions, put `# pytest: python>=x.y` where x and y represent major and minor versions, respectively,
    required to run the test file.
    """
    if path.isfile() and path.ext in (".py", ".scenic"):
        firstLine = ""
        with path.open() as f:
            firstLine = f.readline()
        m = re.search(
            r"^#\s*pytest:\s*python\s*>=\s*(?P<major>\d+)\.(?P<minor>\d+)\s*$", firstLine
        )
        if m:
            target = (int(m.group("major")), int(m.group("minor")))
            return True if sys.version_info < target else None
    return None
