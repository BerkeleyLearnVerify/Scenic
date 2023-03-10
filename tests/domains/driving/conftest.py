
import glob
import os
import shutil

import pytest

from scenic.domains.driving.roads import Network

maps = glob.glob('tests/formats/opendrive/maps/**/*.xodr')

# TODO fix handling of this problematic map
badmap = 'tests/formats/opendrive/maps/opendrive.org/sample1.1.xodr'
map_params = []
for path in maps:
    if path == badmap:
        param = pytest.param(badmap, marks=pytest.mark.xfail(
                    reason='unsolved bug in geometry calculations', strict=True))
    else:
        param = path
    map_params.append(param)

@pytest.fixture(scope='session')
def cached_maps(tmpdir_factory):
    folder = tmpdir_factory.mktemp('maps')
    paths = {}
    for localMap in maps:
        newPath = folder.join(localMap)
        os.makedirs(newPath.dirname, exist_ok=True)
        shutil.copyfile(localMap, newPath)
        paths[localMap] = newPath
    return paths

@pytest.fixture(scope='session')
def network(cached_maps):
    path = cached_maps['tests/formats/opendrive/maps/CARLA/Town03.xodr']
    return Network.fromFile(path)
