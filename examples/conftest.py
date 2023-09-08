"""Set up pytest to check that all examples at least run."""

import types
import warnings

import pytest

import scenic


@pytest.fixture
def options():
    """Fixture defining keyword options used when compiling examples in this folder.

    Subfolders may define their own `conftest.py` file overriding this fixture to
    add additional options or use `pytest.importorskip` to test dependencies.
    """
    return {}


dirsSeen = set()


def pytest_collect_file(parent, file_path):
    # If this is a Scenic file, create a ScenicFolder collector for its parent
    # directory if there isn't one already.
    # (We could just make a collector for each file, but this method groups the
    # tests by folder.)
    folder = file_path.parent
    if file_path.suffix == ".scenic" and folder not in dirsSeen:
        dirsSeen.add(folder)
        return ScenicFolder.from_parent(parent, path=folder)
    return None


class ScenicFolder(pytest.File):
    def collect(self):
        for file in self.path.glob("*.scenic"):
            yield self.itemForFile(file)

    def itemForFile(self, path):
        # Create one test item for this file, as if we had defined the following
        # test function (which uses the `options` fixture defined above).
        def prototype(options):
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                scenario = scenic.scenarioFromFile(path, **options)
                scenario.generate(maxIterations=100_000)

        name = path.name
        node = HackedFunction.from_parent(self, name=name, callobj=prototype)
        return node


class HackedFunction(pytest.Function):
    @property
    def module(self):
        # Use path as "module name" so that pytest-randomly groups by folder.
        fakeModule = types.SimpleNamespace()
        fakeModule.__name__ = self.path
        return fakeModule
