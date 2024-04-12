
import pytest

from verifai.samplers.scenic_sampler import scenicMajorVersion

@pytest.fixture(scope='session')
def new_Object():
    return 'new Object' if scenicMajorVersion >= 3 else 'Object'

def pytest_collection_modifyitems(config, items):
    if scenicMajorVersion < 3:
        mark = pytest.mark.skip(reason='test requires Scenic 3')
        for item in items:
            if (item.module.__name__.startswith('tests.scenic')
                and 'new_Object' not in item.fixturenames):
                item.add_marker(mark)
