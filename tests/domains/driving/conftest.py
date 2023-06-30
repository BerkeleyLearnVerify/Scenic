import glob
import os
from pathlib import Path
import shutil

import pytest

from scenic.domains.driving.roads import Network

mapFolder = Path("assets") / "maps"
maps = glob.glob(str(mapFolder / "**" / "*.xodr"))

# TODO fix handling of this problematic map
badmap = str(mapFolder / "opendrive.org" / "sample1.1.xodr")
map_params = []
for path in maps:
    if path == badmap:
        param = pytest.param(
            badmap,
            marks=pytest.mark.xfail(
                reason="unsolved bug in geometry calculations", strict=True
            ),
        )
    else:
        param = path
    map_params.append(param)


@pytest.fixture(scope="session")
def cached_maps(tmpdir_factory):
    folder = tmpdir_factory.mktemp("maps")
    paths = {}
    for localMap in maps:
        newPath = folder.join(localMap)
        os.makedirs(newPath.dirname, exist_ok=True)
        shutil.copyfile(localMap, newPath)
        paths[localMap] = newPath
    return paths


@pytest.fixture(scope="session")
def network(cached_maps, pytestconfig):
    if pytestconfig.getoption("--fast", False):
        path = mapFolder / "CARLA" / "Town01.xodr"
    else:
        path = mapFolder / "CARLA" / "Town03.xodr"
    path = cached_maps[str(path)]
    return Network.fromFile(path)
