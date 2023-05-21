"""Set up pytest to check that all examples at least run."""

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

def pytest_collect_file(parent, file_path):
    # Create a ScenicFile collector for each Scenic file.
    if file_path.suffix == '.scenic':
        return ScenicFile.from_parent(parent, path=file_path)

class ScenicFile(pytest.File):
    def collect(self):
        # Create one test item for this file, as if we had defined the following
        # test function (which uses the `options` fixture defined above).
        def prototype(options):
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                scenario = scenic.scenarioFromFile(self.path, **options)
                scenario.generate(maxIterations=100_000)

        name = self.path.name
        node = pytest.Function.from_parent(self, name=name, callobj=prototype)
        node._obj = prototype
        yield node
