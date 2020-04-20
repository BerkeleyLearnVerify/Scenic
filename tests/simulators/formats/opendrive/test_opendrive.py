
import os
import glob
import pytest

from scenic.simulators.formats.opendrive import OpenDriveWorkspace

oldDir = os.getcwd()
os.chdir('tests/simulators/formats/opendrive')
maps = glob.glob('maps/**/*.xodr')
os.chdir(oldDir)

@pytest.mark.parametrize("path", maps)
def test_map(path, runLocally):
    with runLocally():
        OpenDriveWorkspace(path, n=10)
