
import os
import glob
import pytest
import shutil
import inspect

from tests.utils import compileScenic, sampleScene, sampleEgo
from scenic.core.geometry import TriangulationError
from scenic.core.distributions import RejectionException

# Suppress all warnings from OpenDRIVE parser
pytestmark = pytest.mark.filterwarnings("ignore::scenic.formats.opendrive.OpenDriveWarning")

template = inspect.cleandoc("""
    param map = '{map}'
    param map_options = dict(useCache={cache})
    model scenic.domains.driving.model
""")

basicScenario = inspect.cleandoc("""
    lane = Uniform(*network.lanes)
    ego = Car in lane
    Car on visible lane.centerline
""")

def compileDrivingScenario(cached_maps, code='', useCache=True,
                           path='tests/formats/opendrive/maps/CARLA/Town01.xodr'):
    path = cached_maps[path]
    preamble = template.format(map=path, cache=useCache)
    whole = preamble + '\n' + inspect.cleandoc(code)
    return compileScenic(whole)

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

@pytest.mark.slow
@pytest.mark.parametrize("path", map_params)
def test_opendrive(path, cached_maps):
    try:
        # First, try the original .xodr file
        scenario = compileDrivingScenario(cached_maps, path=path,
                                          code=basicScenario, useCache=False)
        sampleScene(scenario, maxIterations=1000)
        # Second, try the cached version of the network
        scenario = compileDrivingScenario(cached_maps, path=path,
                                          code=basicScenario, useCache=True)
        sampleScene(scenario, maxIterations=1000)
    except TriangulationError:
        pytest.skip('need better triangulation library to run this test')

def test_elements_at(cached_maps):
    scenario = compileDrivingScenario(cached_maps, """
        ego = Car
        posTuple = (ego.position.x, ego.position.y)
        # functions should accept Points, Vectors, and tuples
        for spot in (ego, ego.position, posTuple):
            param element = network.elementAt(spot)
            param road = network.roadAt(spot)
            param lane = network.laneAt(spot)
            param laneSection = network.laneSectionAt(spot)
            param laneGroup = network.laneGroupAt(spot)
            param crossing = network.crossingAt(spot)
            param intersection = network.intersectionAt(spot)
    """)
    scene = sampleScene(scenario, maxIterations=1000)
    ego = scene.egoObject
    for param in ('element', 'road', 'lane', 'laneSection', 'laneGroup', 'crossing',
                  'intersection'):
        val = scene.params[param]
        if val is None:
            with pytest.raises(RejectionException):
                getattr(ego, param)
        else:
            assert val is getattr(ego, param), param

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

def test_caching(cached_maps):
    """Test caching of road networks.

    In particular, make sure that links between network elements and maneuvers
    are properly reconnected after unpickling.
    """
    for cache in (False, True):
        scenario = compileDrivingScenario(cached_maps, """
            lanes = filter(lambda l: l._successor, network.lanes)
            lane = Uniform(*lanes)
            ego = Car on lane, with foo lane.network.lanes
            Car on ego.lane.successor.centerline, with requireVisible False
            Car on ego.lane.maneuvers[0].endLane.centerline, with requireVisible False
        """, useCache=cache,
        path='tests/formats/opendrive/maps/opendrive.org/CulDeSac.xodr')
        sampleScene(scenario, maxIterations=1000)
