
import os
import glob
import pytest
import shutil

from tests.utils import compileScenic, sampleScene, sampleEgo
from scenic.core.geometry import TriangulationError

template = """
        from scenic.simulators.domains.driving.network import loadNetwork
        loadNetwork('{map}', useCache={cache})
        from scenic.simulators.domains.driving.model import *
        {firstLine}
"""

def compileDrivingScenario(cached_maps, code='', firstLine='', useCache=True,
                           path='tests/simulators/formats/opendrive/maps/CARLA/Town01.xodr'):
    path = cached_maps[path]
    preamble = template.format(map=path, firstLine=firstLine, cache=useCache)
    return compileScenic(preamble + code)

maps = glob.glob('tests/simulators/formats/opendrive/maps/**/*.xodr')

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

@pytest.mark.parametrize("path", maps)
def test_opendrive(path, cached_maps):
    try:
        # First, try the original .xodr file
        scenario = compileDrivingScenario(cached_maps, path=path,
                                          firstLine='ego = Car', useCache=False)
        sampleScene(scenario, maxIterations=1000)
        # Second, try the cached version of the network
        scenario = compileDrivingScenario(cached_maps, path=path,
                                          firstLine='ego = Car', useCache=True)
        sampleScene(scenario, maxIterations=1000)
    except TriangulationError:
        pytest.skip('need better triangulation library to run this test')

def test_elements_at(cached_maps):
    scenario = compileDrivingScenario(cached_maps, """
        ego = Car
        param element = network.elementAt(ego)
        param road = network.roadAt(ego)
        param lane = network.laneAt(ego)
        param laneSection = network.laneSectionAt(ego)
        param laneGroup = network.laneGroupAt(ego)
        param crossing = network.crossingAt(ego)
        param intersection = network.intersectionAt(ego)
    """)
    scene = sampleScene(scenario, maxIterations=1000)
    ego = scene.egoObject
    for param in ('element', 'road', 'lane', 'laneSection', 'laneGroup', 'crossing',
                  'intersection'):
        assert scene.params[param] is getattr(ego, param), param

def test_intersection(cached_maps):
    scenario = compileDrivingScenario(cached_maps, """
        intersection = Uniform(*network.intersections)
        lane = Uniform(*intersection.incomingLanes)
        maneuver = Uniform(*lane.maneuvers)
        ego = Car on maneuver.connectingLane.centerline
    """)
    for i in range(50):
        ego = sampleEgo(scenario, maxIterations=1000)
        intersection = ego.intersection
        assert intersection is not None
        directions = intersection.nominalDirectionsAt(ego)
        assert any(ego.heading == pytest.approx(direction) for direction in directions)

def test_curb(cached_maps):
    scenario = compileDrivingScenario(cached_maps, """
        ego = Car
        spot = OrientedPoint on visible curb
        Car left of spot by 0.25
    """)
    sampleScene(scenario, maxIterations=1000)
