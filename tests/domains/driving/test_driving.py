import inspect
import os
import shutil

import pytest

from scenic.core.distributions import RejectionException
from scenic.core.errors import InvalidScenarioError
from scenic.core.geometry import TriangulationError
from scenic.domains.driving.roads import Network
from tests.utils import compileScenic, pickle_test, sampleEgo, sampleScene, tryPickling

# Suppress all warnings from OpenDRIVE parser
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.formats.opendrive.OpenDriveWarning"
)

template = inspect.cleandoc(
    """
    param map = r'{map}'
    param map_options = dict(useCache={cache})
    model scenic.domains.driving.model
"""
)

basicScenario = inspect.cleandoc(
    """
    lane = Uniform(*network.lanes)
    ego = new Car in lane
    new Car on visible lane.centerline
"""
)

from tests.domains.driving.conftest import map_params, mapFolder


def compileDrivingScenario(
    cached_maps, code="", useCache=True, path=None, mode2D=False, params={}
):
    if not path:
        path = mapFolder / "CARLA" / "Town01.xodr"
    path = cached_maps[str(path)]
    preamble = template.format(map=path, cache=useCache)
    whole = preamble + "\n" + inspect.cleandoc(code)
    return compileScenic(whole, mode2D=mode2D, params=params)


def test_driving_2D_map(cached_maps):
    compileDrivingScenario(
        cached_maps,
        code=basicScenario,
        useCache=False,
        mode2D=False,
        params={"use2DMap": True},
    )


def test_driving_3D(cached_maps3D):
    compileDrivingScenario(
        cached_maps3D, code=basicScenario, useCache=False, mode2D=False
    )


@pytest.mark.slow
@pytest.mark.parametrize("path", map_params)
@pytest.mark.parametrize("use2DMap", [False, True])
def test_opendrive(path, cached_maps, cached_maps3D, use2DMap):
    try:
        # First, try the original .xodr file
        scenario = compileDrivingScenario(
            cached_maps if use2DMap else cached_maps3D,
            path=path,
            code=basicScenario,
            useCache=False,
            mode2D=use2DMap,
        )
        sampleScene(scenario, maxIterations=1000)
        # Second, try the cached version of the network
        scenario = compileDrivingScenario(
            cached_maps if use2DMap else cached_maps3D,
            path=path,
            code=basicScenario,
            useCache=True,
            mode2D=use2DMap,
        )
        sampleScene(scenario, maxIterations=1000)
    except TriangulationError:
        pytest.skip("need better triangulation library to run this test")


@pytest.mark.parametrize("use2DMap", [True, False])
def test_elements_at(cached_maps, cached_maps3D, use2DMap):
    scenario = compileDrivingScenario(
        cached_maps if use2DMap else cached_maps3D,
        """
        ego = new Car
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
    """,
        mode2D=use2DMap,
    )
    scene = sampleScene(scenario, maxIterations=1000)
    ego = scene.egoObject
    for param in (
        "element",
        "road",
        "lane",
        "laneSection",
        "laneGroup",
        "crossing",
        "intersection",
    ):
        val = scene.params[param]
        if val is None:
            with pytest.raises(RejectionException):
                getattr(ego, param)
        else:
            assert val is getattr(ego, param), param


@pytest.mark.parametrize("use2DMap", [True, False])
def test_intersection(cached_maps, cached_maps3D, use2DMap):
    scenario = compileDrivingScenario(
        cached_maps if use2DMap else cached_maps3D,
        """
        intersection = Uniform(*network.intersections)
        lane = Uniform(*intersection.incomingLanes)
        maneuver = Uniform(*lane.maneuvers)
        ego = new Car on maneuver.connectingLane.centerline
    """,
        mode2D=use2DMap,
    )
    for i in range(20):
        ego = sampleEgo(scenario, maxIterations=1000)
        intersection = ego.intersection
        assert intersection is not None
        assert intersection.is3Way == (len(intersection.roads) == 3)
        assert intersection.is4Way == (len(intersection.roads) == 4)
        assert intersection.isSignalized == bool(intersection.signals)
        network = intersection.network
        assert network.elementAt(ego) is intersection
        directions = intersection.nominalDirectionsAt(ego)
        maneuvers = intersection.maneuversAt(ego)
        lane = ego.lane
        assert directions == network.nominalDirectionsAt(ego)
        assert any(
            ego.heading == pytest.approx(direction.yaw) for direction in directions
        )

        assert any(man.connectingLane is lane for man in maneuvers)


# Tests end here
@pytest.mark.parametrize("use2DMap", [True, False])
def test_curb(cached_maps, cached_maps3D, use2DMap):

    scenario = compileDrivingScenario(
        cached_maps if use2DMap else cached_maps3D,
        """
        ego = new Car
        spot = new OrientedPoint on visible curb
        new Car left of spot by 0.25
    """,
        mode2D=use2DMap,
    )
    ego = sampleEgo(scenario, maxIterations=1000)
    directions = ego.element.network.nominalDirectionsAt(ego)
    assert any(ego.heading == pytest.approx(direction.yaw) for direction in directions)


@pytest.mark.slow
@pytest.mark.parametrize("use2DMap", [True, False])
def test_caching(tmpdir, use2DMap):
    """Test caching of road networks.

    In particular, make sure that links between network elements and maneuvers
    are properly reconnected after unpickling.
    """
    origMap = mapFolder / "CARLA" / "Town01.xodr"
    path = os.path.join(tmpdir, "map.xodr")
    cachedPath = os.path.join(tmpdir, "map" + Network.pickledExt)
    noExtPath = os.path.join(tmpdir, "map")
    shutil.copyfile(origMap, path)
    opts = (
        (False, path),
        (True, cachedPath),
        (True, noExtPath),
        (True, path),
    )
    for useCache, path in opts:
        scenario = compileScenic(
            f"""
            param map = r'{path}'
            param map_options = dict(useCache={useCache})
            model scenic.domains.driving.model
            lanes = filter(lambda l: l._successor, network.lanes)
            lane = Uniform(*lanes)
            ego = new Car on lane, with foo lane.network.lanes
            new Car on ego.lane.successor.centerline, with requireVisible False
            new Car on ego.lane.maneuvers[0].endLane.centerline, with requireVisible False
        """,
            mode2D=use2DMap,
        )
        sampleScene(scenario, maxIterations=1000)


@pickle_test
@pytest.mark.slow
@pytest.mark.parametrize("use2DMap", [True, False])
def test_pickle(cached_maps, cached_maps3D, use2DMap):
    scenario = compileDrivingScenario(
        cached_maps if use2DMap else cached_maps3D,
        """
        ego = new Car with behavior FollowLaneBehavior(target_speed=Range(10, 15))
        new Pedestrian on visible sidewalk
    """,
        mode2D=use2DMap,
    )
    unpickled = tryPickling(scenario)
    scene = sampleScene(unpickled, maxIterations=1000)
    tryPickling(scene)


@pytest.mark.parametrize("use2DMap", [True, False])
def test_invalid_road_scenario(cached_maps, cached_maps3D, use2DMap):
    with pytest.raises(InvalidScenarioError):
        scenario = compileDrivingScenario(
            cached_maps if use2DMap else cached_maps3D,
            """
            ego = new Car at 80.6354425964952@-327.5431187869811
            param foo = ego.oppositeLaneGroup.sidewalk
            """,
            mode2D=use2DMap,
        )

    with pytest.raises(InvalidScenarioError):
        # Set regionContainedIn to everywhere to hit driving domain specific code
        # instead of high level not contained in workspace rejection.
        scenario = compileDrivingScenario(
            cached_maps if use2DMap else cached_maps3D,
            """
            ego = new Car at 10000@10000, with regionContainedIn everywhere
            param foo = ego.lane
            """,
            mode2D=use2DMap,
        )


def test_cars_at_underpass(cached_maps3D):
    scenario = compileDrivingScenario(
        cached_maps3D,
        """
        ego = new Car on road, at (10, -12, 0), with regionContainedIn everywhere
        """,
        path=mapFolder / "CARLA" / "Town04.xodr",
    )
    ego = sampleEgo(scenario, maxIterations=1000)
    directions = ego.element.network.nominalDirectionsAt(ego)
    # Check the car is under the overpass
    assert ego.position.x > 8 and ego.position.x < 12
    assert ego.position.y < -10 and ego.position.y > -14
    assert ego.position.z < 5
    # Checks that road orientation is same as ego orientation
    assert any(ego.heading == pytest.approx(direction.yaw) for direction in directions)
    assert any(
        ego.orientation.pitch == pytest.approx(direction.pitch)
        for direction in directions
    )
    assert any(
        ego.orientation.roll == pytest.approx(direction.roll) for direction in directions
    )


def test_cars_at_overpass(cached_maps3D):
    scenario = compileDrivingScenario(
        # (10, -12, 100) projects car down to map
        cached_maps3D,
        """
        ego = new Car on road, at (10, -12, 100), with regionContainedIn everywhere
        """,
        path=mapFolder / "CARLA" / "Town04.xodr",
    )
    ego = sampleEgo(scenario, maxIterations=1000)
    directions = ego.element.network.nominalDirectionsAt(ego)
    # Check the car is over the overpass
    assert ego.position.x > 8 and ego.position.x < 12
    assert ego.position.y < -10 and ego.position.y > -14
    assert ego.position.z > 10
    # Checks that road orientation is same as ego orientation
    assert any(ego.heading == pytest.approx(direction.yaw) for direction in directions)
    assert any(
        ego.orientation.pitch == pytest.approx(direction.pitch)
        for direction in directions
    )
    assert any(
        ego.orientation.roll == pytest.approx(direction.roll) for direction in directions
    )


def test_car_on_slope(cached_maps3D):
    scenario = compileDrivingScenario(
        cached_maps3D,
        """
        ego = new Car on road, at (200, -12, 0), with regionContainedIn everywhere
        """,
        path=mapFolder / "CARLA" / "Town04.xodr",
    )
    ego = sampleEgo(scenario, maxIterations=1000)
    directions = ego.element.network.nominalDirectionsAt(ego)
    # Checks that road orientation is same as ego orientation
    assert any(ego.heading == pytest.approx(direction.yaw) for direction in directions)
    assert any(
        ego.orientation.pitch == pytest.approx(direction.pitch)
        for direction in directions
    )
    assert any(
        ego.orientation.roll == pytest.approx(direction.roll) for direction in directions
    )
