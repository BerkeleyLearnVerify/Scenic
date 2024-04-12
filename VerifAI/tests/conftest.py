
import os.path
import pytest

# Change matplotlib backend to fix problems testing in a virtualenv
# (on macOS, the native backend won't work and will break importing GPyOpt)
import matplotlib
matplotlib.use('Agg')

@pytest.fixture
def pathToLocalFile(request):
    base = os.path.dirname(request.fspath)
    def pathFinder(relpath):
        return os.path.join(base, relpath)
    return pathFinder
