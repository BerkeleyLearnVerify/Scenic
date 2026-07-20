"""Left-hand traffic OpenDRIVE maps — centerlines and faster/slower lanes.

These tests exercise ``assets/maps/demo/lht`` only. They do not change the
OpenDRIVE parser: with the current Scenic loader, one-way LHT maps that place
driving lanes on positive OpenDRIVE IDs still yield a consistent traffic flow
and median/curb faster-slower relationships after centerlines are oriented for
driving.

Absolute OpenDRIVE LHT conventions (positive IDs travel +s, oncoming to the
driver's right) are intentionally not asserted here.
"""

from pathlib import Path

import pytest

from scenic.domains.driving.roads import Network

pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.formats.opendrive.OpenDriveWarning"
)

DEMO = Path(__file__).resolve().parents[3] / "assets" / "maps" / "demo" / "lht"


def load(name):
    return Network.fromFile(DEMO / name, useCache=False)


def section_by_id(network, od_id):
    for road in network.roads:
        for sec in road.sections:
            if od_id in sec.lanesByOpenDriveID:
                return sec.lanesByOpenDriveID[od_id]
    raise AssertionError(f"no lane section with OpenDRIVE id {od_id}")


def centerline_delta(lane):
    pts = lane.centerline.points
    return pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]


def carriageway_lanes(road):
    """Yield each same-direction lane group on the road."""
    if road.forwardLanes:
        yield road.forwardLanes
    if road.backwardLanes:
        yield road.backwardLanes


# ---------------------------------------------------------------------------
# Maps load
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name, n_lanes, oneway",
    [
        ("01_two_lane_oneway.xodr", 2, True),
        ("02_three_lane_speeds.xodr", 3, True),
        ("03_two_way.xodr", 4, False),
    ],
)
def test_lht_maps_load(name, n_lanes, oneway):
    network = load(name)
    assert len(network.roads) == 1
    road = network.roads[0]
    assert len(network.lanes) == n_lanes
    assert road.is1Way is oneway


# ---------------------------------------------------------------------------
# Centerlines follow the flow of traffic
# ---------------------------------------------------------------------------


def test_centerlines_agree_within_each_carriageway():
    """Every lane in a carriageway points the same way (traffic flow)."""
    for name in (
        "01_two_lane_oneway.xodr",
        "02_three_lane_speeds.xodr",
        "03_two_way.xodr",
    ):
        road = load(name).roads[0]
        for group in carriageway_lanes(road):
            deltas = [centerline_delta(lane) for lane in group.lanes]
            assert deltas, name
            # Same sign in the dominant axis (these maps are axis-aligned on x).
            signs = [1 if dx > 0 else -1 if dx < 0 else 0 for dx, _ in deltas]
            assert 0 not in signs, f"{name}: zero-length centerline"
            assert len(set(signs)) == 1, f"{name}: mixed centerline directions {deltas}"


def test_centerline_matches_lane_orientation():
    """Lane orientation at a centerline point matches the network flow direction."""
    network = load("02_three_lane_speeds.xodr")
    for lane in network.lanes:
        pt = lane.centerline.pointAlongBy(0.5, normalized=True)
        dirs = network.nominalDirectionsAt(pt)
        assert pytest.approx(lane.orientation[pt]) in dirs
        assert lane.containsPoint(pt)


def test_two_way_carriageways_oppose_each_other():
    """Opposite carriageways on a two-way road travel opposite directions."""
    road = load("03_two_way.xodr").roads[0]
    assert road.forwardLanes and road.backwardLanes
    fwd = centerline_delta(road.forwardLanes.lanes[0])[0]
    bwd = centerline_delta(road.backwardLanes.lanes[0])[0]
    assert fwd * bwd < 0


# ---------------------------------------------------------------------------
# fasterLane / slowerLane point toward median / curb
# ---------------------------------------------------------------------------


def test_two_lane_faster_slower_toward_median():
    """Curb lane's faster neighbor is the median lane; median's slower is curb."""
    network = load("01_two_lane_oneway.xodr")
    median = section_by_id(network, 1)
    curb = section_by_id(network, 2)

    assert curb._fasterLane is median
    assert median._slowerLane is curb
    assert curb._slowerLane is None
    assert median._fasterLane is None

    assert curb.fasterLane is median
    assert median.slowerLane is curb


def test_three_lane_faster_slower_chain():
    """Curb → middle → median is the faster direction; reverse is slower."""
    network = load("02_three_lane_speeds.xodr")
    median = section_by_id(network, 1)
    middle = section_by_id(network, 2)
    curb = section_by_id(network, 3)

    assert curb._fasterLane is middle
    assert middle._slowerLane is curb
    assert middle._fasterLane is median
    assert median._slowerLane is middle
    assert curb._slowerLane is None
    assert median._fasterLane is None


def test_two_way_faster_slower_each_carriageway():
    """Each side of a two-way LHT road: outer/curb is slower, inner/median faster."""
    network = load("03_two_way.xodr")

    # Positive (left) carriageway: +2 curb, +1 median
    pos_curb = section_by_id(network, 2)
    pos_med = section_by_id(network, 1)
    assert pos_curb._fasterLane is pos_med
    assert pos_med._slowerLane is pos_curb
    assert pos_med._fasterLane is None

    # Negative (right) carriageway: -2 curb, -1 median
    neg_curb = section_by_id(network, -2)
    neg_med = section_by_id(network, -1)
    assert neg_curb._fasterLane is neg_med
    assert neg_med._slowerLane is neg_curb
    assert neg_med._fasterLane is None


def test_faster_slower_are_same_direction_neighbors():
    """faster/slower never point across the median to oncoming traffic."""
    for name in (
        "01_two_lane_oneway.xodr",
        "02_three_lane_speeds.xodr",
        "03_two_way.xodr",
    ):
        network = load(name)
        for road in network.roads:
            for sec in road.sections:
                for lane in sec.lanes:
                    if lane._fasterLane:
                        assert lane._fasterLane.isForward == lane.isForward
                    if lane._slowerLane:
                        assert lane._slowerLane.isForward == lane.isForward


# ---------------------------------------------------------------------------
# Geometry integrity
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name",
    [
        "01_two_lane_oneway.xodr",
        "02_three_lane_speeds.xodr",
        "03_two_way.xodr",
    ],
)
def test_lane_polygons_valid_and_contain_centerline(name):
    network = load(name)
    for lane in network.lanes:
        assert not lane.polygon.is_empty
        assert lane.polygon.is_valid
        for sec in lane.sections:
            assert sec.containsRegion(sec.centerline, tolerance=0.5)
            assert sec.containsRegion(sec.leftEdge, tolerance=0.5)
            assert sec.containsRegion(sec.rightEdge, tolerance=0.5)


# ---------------------------------------------------------------------------
# Adjacency helpers (shiftedBy, opposite group, edge rejection)
# ---------------------------------------------------------------------------


def test_shifted_by_matches_lane_to_left_right():
    network = load("02_three_lane_speeds.xodr")
    for sec in network.roads[0].sections[0].lanes:
        if sec._laneToLeft:
            assert sec.shiftedBy(1) is sec._laneToLeft
            assert sec._laneToLeft.shiftedBy(-1) is sec
        if sec._laneToRight:
            assert sec.shiftedBy(-1) is sec._laneToRight
            assert sec._laneToRight.shiftedBy(1) is sec


def test_faster_slower_are_left_or_right_neighbors():
    network = load("02_three_lane_speeds.xodr")
    for sec in network.roads[0].sections[0].lanes:
        neighbors = {sec._laneToLeft, sec._laneToRight} - {None}
        if sec._fasterLane:
            assert sec._fasterLane in neighbors
        if sec._slowerLane:
            assert sec._slowerLane in neighbors


def test_two_way_opposite_lane_groups():
    road = load("03_two_way.xodr").roads[0]
    assert road.forwardLanes._opposite is road.backwardLanes
    assert road.backwardLanes._opposite is road.forwardLanes
    assert road.forwardLanes.opposite is road.backwardLanes
    assert road.backwardLanes.opposite is road.forwardLanes


def test_edge_lanes_reject_missing_faster_or_slower():
    from scenic.core.distributions import RejectionException

    network = load("01_two_lane_oneway.xodr")
    median = section_by_id(network, 1)
    curb = section_by_id(network, 2)

    with pytest.raises(RejectionException):
        _ = median.fasterLane
    with pytest.raises(RejectionException):
        _ = curb.slowerLane
