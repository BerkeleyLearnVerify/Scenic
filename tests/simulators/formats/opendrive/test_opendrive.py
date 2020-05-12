
import os
import glob
import pytest

from scenic.simulators.formats.opendrive import OpenDriveWorkspace
from scenic.core.geometry import TriangulationError

oldDir = os.getcwd()
os.chdir('tests/simulators/formats/opendrive')
maps = glob.glob('maps/**/*.xodr')
os.chdir(oldDir)

@pytest.mark.parametrize("path", maps)
def test_map(path, runLocally):
    with runLocally():
        try:
            OpenDriveWorkspace(path, n=10)
        except TriangulationError:
            pytest.skip('need better triangulation library to run this test')
