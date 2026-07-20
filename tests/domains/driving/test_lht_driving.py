"""Driving-domain smoke tests on left-hand traffic demo maps."""

import inspect
from pathlib import Path

import pytest

from tests.utils import compileScenic, sampleEgo, sampleScene

pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.formats.opendrive.OpenDriveWarning"
)

LHT = Path(__file__).resolve().parents[3] / "assets" / "maps" / "demo" / "lht"

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
    new Car on visible lane.centerline
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


def _compile(path, code):
    preamble = template.format(map=path)
    return compileScenic(preamble + "\n" + code, mode2D=True)


@pytest.mark.parametrize(
    "name",
    [
        "01_two_lane_oneway.xodr",
        "02_three_lane_speeds.xodr",
        "03_two_way.xodr",
    ],
)
def test_lht_driving_scenario_compiles(name):
    scenario = _compile(LHT / name, basicScenario)
    sampleScene(scenario, maxIterations=1000)


@pytest.mark.parametrize(
    "name",
    [
        "01_two_lane_oneway.xodr",
        "02_three_lane_speeds.xodr",
        "03_two_way.xodr",
    ],
)
def test_lht_faster_lane_placement(name):
    """Cars can be placed on a lane and on that lane's fasterLane."""
    scenario = _compile(LHT / name, fasterLaneScenario)
    ego = sampleEgo(scenario, maxIterations=1000)
    assert ego.lane is not None


@pytest.mark.parametrize(
    "name",
    [
        "01_two_lane_oneway.xodr",
        "02_three_lane_speeds.xodr",
        "03_two_way.xodr",
    ],
)
def test_lht_slower_lane_placement(name):
    """Cars can be placed on a lane and on that lane's slowerLane."""
    scenario = _compile(LHT / name, slowerLaneScenario)
    ego = sampleEgo(scenario, maxIterations=1000)
    assert ego.lane is not None
