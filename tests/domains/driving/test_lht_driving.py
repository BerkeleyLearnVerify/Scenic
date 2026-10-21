"""Driving-domain tests on left-hand traffic demo maps."""

import inspect
from pathlib import Path

import pytest

from tests.utils import compileScenic, sampleScene

pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.formats.opendrive.OpenDriveWarning"
)

MAPS = Path(__file__).resolve().parents[3] / "assets" / "maps"

CORRIDORS = (
    "left-hand-traffic/handmade_oneway_two_lane.xodr",
    "left-hand-traffic/handmade_oneway_three_lane.xodr",
    "left-hand-traffic/handmade_two_way.xodr",
    "left-hand-traffic/handmade_north_south.xodr",
)

JUNCTIONS = (
    "left-hand-traffic/handmade_t_junction.xodr",
    "left-hand-traffic/handmade_two_way_junction.xodr",
)

template = inspect.cleandoc(
    """
    param map = r'{map}'
    param map_options = dict(useCache=False)
    model scenic.domains.driving.model
    """
)

basicScenario = inspect.cleandoc(
    """
    lane = Uniform(*network.lanes)
    ego = new Car in lane
    follower = new Car on visible lane.centerline
    """
)

fasterLaneScenario = inspect.cleandoc(
    """
    candidates = [lane for lane in network.lanes if lane.sections[0]._fasterLane]
    require len(candidates) > 0
    lane = Uniform(*candidates)
    ego = new Car in lane
    other = new Car in lane.sections[0].fasterLane.lane
    """
)

slowerLaneScenario = inspect.cleandoc(
    """
    candidates = [lane for lane in network.lanes if lane.sections[0]._slowerLane]
    require len(candidates) > 0
    lane = Uniform(*candidates)
    ego = new Car in lane
    other = new Car in lane.sections[0].slowerLane.lane
    """
)

maneuverScenario = inspect.cleandoc(
    """
    require len(network.intersections) > 0
    maneuver = Uniform(*network.intersections[0].maneuvers)
    ego = new Car in maneuver.startLane
    other = new Car in maneuver.endLane
    """
)


def _compile(path, code):
    preamble = template.format(map=path)
    return compileScenic(preamble + "\n" + code, mode2D=True)


def _sample_two_car_scene(rel, scenario_code):
    scenario = _compile(MAPS / rel, scenario_code)
    scene = sampleScene(scenario, maxIterations=1000)
    assert len(scene.objects) == 2
    ego, other = scene.objects
    assert ego is scene.egoObject
    return ego, other


@pytest.mark.parametrize("rel", CORRIDORS + JUNCTIONS)
def test_basic_car_placement(rel):
    """Ego and a second car can be placed on the same sampled lane."""
    ego, follower = _sample_two_car_scene(rel, basicScenario)

    assert ego.lane is follower.lane
    assert ego.laneSection.lane is follower.laneSection.lane
    assert follower.lane.containsPoint(follower.position)


@pytest.mark.parametrize("rel", CORRIDORS)
def test_faster_lane_placement(rel):
    """Ego sits on a slower lane; other is placed on its fasterLane neighbor."""
    ego, other = _sample_two_car_scene(rel, fasterLaneScenario)

    ego_sec = ego.laneSection
    assert ego_sec._fasterLane is not None
    assert other.lane is ego_sec.fasterLane.lane
    assert other.lane is not ego.lane
    assert other.laneSection.lane is ego_sec.fasterLane.lane


@pytest.mark.parametrize("rel", CORRIDORS)
def test_slower_lane_placement(rel):
    """Ego sits on a faster lane; other is placed on its slowerLane neighbor."""
    ego, other = _sample_two_car_scene(rel, slowerLaneScenario)

    ego_sec = ego.laneSection
    assert ego_sec._slowerLane is not None
    assert other.lane is ego_sec.slowerLane.lane
    assert other.lane is not ego.lane
    assert other.laneSection.lane is ego_sec.slowerLane.lane


@pytest.mark.parametrize("rel", JUNCTIONS)
def test_junction_maneuver_placement(rel):
    """Cars can sit on the start and end lanes of a sampled junction maneuver."""
    ego, other = _sample_two_car_scene(rel, maneuverScenario)
    assert other.lane is not ego.lane
    matching = [m for m in ego.lane.maneuvers if m.endLane is other.lane]
    assert matching
