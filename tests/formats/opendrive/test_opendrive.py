
import os
import glob

import matplotlib.pyplot as plt
import pytest

from scenic.formats.opendrive import OpenDriveWorkspace
from scenic.core.geometry import TriangulationError

oldDir = os.getcwd()
os.chdir('tests/formats/opendrive')
maps = glob.glob('maps/**/*.xodr')
os.chdir(oldDir)

@pytest.mark.slow
@pytest.mark.filterwarnings("ignore::scenic.formats.opendrive.OpenDriveWarning")
@pytest.mark.parametrize("path", maps)
def test_map(path, runLocally):
    with runLocally():
        try:
            odw = OpenDriveWorkspace(path, n=10)
        except TriangulationError:
            pytest.skip('need better triangulation library to run this test')
        pt = odw.drivable_region.uniformPointInner()
        odw.road_direction[pt]
        odw.show(plt)
        plt.show(block=False)
        plt.close()
