import glob
import os
from pathlib import Path

import matplotlib.pyplot as plt
import pytest

from scenic.core.geometry import TriangulationError
from scenic.formats.opendrive import OpenDriveWorkspace
from tests.formats.opendrive.conftest import maps


@pytest.mark.slow
@pytest.mark.parametrize("path", maps)
@pytest.mark.filterwarnings("ignore::scenic.formats.opendrive.OpenDriveWarning")
def test_map(path, pytestconfig):
    try:
        odw = OpenDriveWorkspace(path, n=10)
    except TriangulationError:
        pytest.skip("need better triangulation library to run this test")
    pt = odw.drivable_region.uniformPointInner()
    odw.road_direction[pt]
    if not pytestconfig.getoption("--no-graphics"):
        odw.show(plt)
        plt.show(block=False)
        plt.close()
