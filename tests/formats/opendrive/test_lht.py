"""Left-hand traffic OpenDRIVE maps — centerlines and faster/slower lanes."""

import math
from pathlib import Path

import pytest

from scenic.core.distributions import RejectionException
from scenic.core.geometry import normalizeAngle
from scenic.core.vectors import Vector
from scenic.domains.driving.roads import Network
from scenic.formats.opendrive.xodr_parser import OpenDriveWarning

pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.formats.opendrive.OpenDriveWarning"
)

MAPS = Path(__file__).resolve().parents[3] / "assets" / "maps"

# (relative path, lane count, one-way)
CORRIDORS = (
    ("left-hand-traffic/handmade_oneway_two_lane.xodr", 2, True),
    ("left-hand-traffic/handmade_oneway_three_lane.xodr", 3, True),
    ("left-hand-traffic/handmade_two_way.xodr", 4, False),
    ("left-hand-traffic/handmade_north_south.xodr", 4, False),
)

CORRIDOR_PATHS = tuple(path for path, *_ in CORRIDORS)

JUNCTIONS = (
    "left-hand-traffic/handmade_t_junction.xodr",
    "left-hand-traffic/handmade_two_way_junction.xodr",
)


def load(rel):
    return Network.fromFile(MAPS / rel, useCache=False)


def section_by_id(network, od_id):
    for road in network.roads:
        for sec in road.sections:
            if od_id in sec.lanesByOpenDriveID:
                return sec.lanesByOpenDriveID[od_id]
    raise AssertionError(f"no lane section with OpenDRIVE id {od_id}")


def carriageway_groups(road):
    if road.forwardLanes:
        yield road.forwardLanes
    if road.backwardLanes:
        yield road.backwardLanes


def chord_heading(lane):
    """Scenic heading of the centerline from its first point to its last."""
    pts = lane.centerline.points
    p0, p1 = Vector(*pts[0][:2]), Vector(*pts[-1][:2])
    assert p0.distanceTo(p1) > 1e-6, "zero-length centerline"
    return p0.angleTo(p1)


def headings_close(a, b, tol=0.05):
    return abs(normalizeAngle(a - b)) < tol


def assert_faster_slower(network, sec, *, faster_id, slower_id):
    faster = section_by_id(network, faster_id) if faster_id else None
    slower = section_by_id(network, slower_id) if slower_id else None
    assert sec._fasterLane is faster
    assert sec._slowerLane is slower
    if faster_id is None:
        with pytest.raises(RejectionException):
            _ = sec.fasterLane
    else:
        assert sec.fasterLane is faster
    if slower_id is None:
        with pytest.raises(RejectionException):
            _ = sec.slowerLane
    else:
        assert sec.slowerLane is slower


@pytest.mark.parametrize("rel, n_lanes, oneway", CORRIDORS)
def test_network_structure(rel, n_lanes, oneway):
    network = load(rel)
    road = network.roads[0]

    assert len(network.roads) == 1
    assert len(network.lanes) == n_lanes
    assert road.is1Way is oneway
    for sec in road.sections:
        for od_id, lane_sec in sec.lanesByOpenDriveID.items():
            assert lane_sec.isForward is (od_id > 0)

    for lane in network.lanes:
        assert not lane.polygon.is_empty
        assert lane.polygon.is_valid
        for sec in lane.sections:
            assert sec.containsRegion(sec.centerline, tolerance=0.5)
            assert sec.containsRegion(sec.leftEdge, tolerance=0.5)
            assert sec.containsRegion(sec.rightEdge, tolerance=0.5)


@pytest.mark.parametrize("rel", CORRIDOR_PATHS)
def test_centerlines_follow_traffic_flow(rel):
    """Centerlines point with traffic on any heading, not only east-west."""
    network = load(rel)
    road = network.roads[0]

    for group in carriageway_groups(road):
        headings = [chord_heading(lane) for lane in group.lanes]
        for h in headings[1:]:
            assert headings_close(h, headings[0]), f"{rel}: mixed centerline directions"

        for lane in group.lanes:
            pt = lane.centerline.pointAlongBy(0.5, normalized=True)
            dirs = road.network.nominalDirectionsAt(pt)
            assert pytest.approx(lane.orientation[pt]) in dirs
            assert lane.containsPoint(pt)
            assert headings_close(lane.orientation[pt].yaw, chord_heading(lane))

    if not road.is1Way:
        fwd = chord_heading(road.forwardLanes.lanes[0])
        bwd = chord_heading(road.backwardLanes.lanes[0])
        assert headings_close(abs(normalizeAngle(fwd - bwd)), math.pi)


@pytest.mark.parametrize(
    "rel, pairs",
    [
        (
            "left-hand-traffic/handmade_oneway_two_lane.xodr",
            ((2, 1, None), (1, None, 2)),
        ),
        (
            "left-hand-traffic/handmade_oneway_three_lane.xodr",
            ((3, 2, None), (2, 1, 3), (1, None, 2)),
        ),
        (
            "left-hand-traffic/handmade_two_way.xodr",
            ((2, 1, None), (1, None, 2), (-2, -1, None), (-1, None, -2)),
        ),
        (
            "left-hand-traffic/handmade_north_south.xodr",
            ((2, 1, None), (1, None, 2), (-2, -1, None), (-1, None, -2)),
        ),
    ],
)
def test_faster_slower_toward_median(rel, pairs):
    """Faster is toward the reference line (median)."""
    network = load(rel)
    for od_id, faster_id, slower_id in pairs:
        assert_faster_slower(
            network,
            section_by_id(network, od_id),
            faster_id=faster_id,
            slower_id=slower_id,
        )


@pytest.mark.parametrize("rel", CORRIDOR_PATHS)
def test_lane_adjacency_helpers(rel):
    """shiftedBy, faster/slower neighbors, and same-direction constraints."""
    network = load(rel)
    for road in network.roads:
        for sec in road.sections:
            for lane_sec in sec.lanes:
                neighbors = {lane_sec._laneToLeft, lane_sec._laneToRight} - {None}
                if lane_sec._laneToLeft:
                    assert lane_sec.shiftedBy(1) is lane_sec._laneToLeft
                    if lane_sec._laneToLeft.isForward == lane_sec.isForward:
                        assert lane_sec._laneToLeft.shiftedBy(-1) is lane_sec
                        assert lane_sec._fasterLane is lane_sec._laneToLeft
                if lane_sec._laneToRight:
                    assert lane_sec.shiftedBy(-1) is lane_sec._laneToRight
                    if lane_sec._laneToRight.isForward == lane_sec.isForward:
                        assert lane_sec._laneToRight.shiftedBy(1) is lane_sec
                        assert lane_sec._slowerLane is lane_sec._laneToRight
                if lane_sec._fasterLane:
                    assert lane_sec._fasterLane in neighbors
                    assert lane_sec._fasterLane.isForward == lane_sec.isForward
                if lane_sec._slowerLane:
                    assert lane_sec._slowerLane in neighbors
                    assert lane_sec._slowerLane.isForward == lane_sec.isForward

    if rel.endswith("handmade_two_way.xodr") or rel.endswith("handmade_north_south.xodr"):
        road = network.roads[0]
        assert road.forwardLanes._opposite is road.backwardLanes
        assert road.backwardLanes.opposite is road.forwardLanes


@pytest.mark.parametrize("rel", JUNCTIONS)
def test_junction_parses_with_maneuvers(rel):
    network = load(rel)
    assert len(network.intersections) == 1
    inter = network.intersections[0]
    assert inter.maneuvers
    assert inter.incomingLanes
    assert inter.outgoingLanes

    for maneuver in inter.maneuvers:
        start, connecting, end = (
            maneuver.startLane,
            maneuver.connectingLane,
            maneuver.endLane,
        )
        assert connecting is not None
        assert end is not None
        assert start.road is not end.road
        assert connecting.road is not start.road
        # Connecting-lane centerline follows the maneuver, on whatever heading.
        assert headings_close(
            chord_heading(connecting),
            connecting.centerline.orientation[
                connecting.centerline.pointAlongBy(0.5, normalized=True)
            ].yaw,
        )
        assert any(m.connectingLane is connecting for m in start.maneuvers)


def test_t_junction_splits_west_arm():
    network = load("left-hand-traffic/handmade_t_junction.xodr")
    assert {road.id for road in network.roads} == {1, 2, 3}
    ends = {m.endLane.road.id for m in network.intersections[0].maneuvers}
    assert ends == {2, 3}
    for lane in network.lanes:
        for sec in lane.sections:
            assert sec.isForward is (sec.openDriveID > 0)


def test_two_way_junction_has_opposing_connecting_flows():
    network = load("left-hand-traffic/handmade_two_way_junction.xodr")
    maneuvers = list(network.intersections[0].maneuvers)
    assert len(maneuvers) == 2
    headings = sorted(chord_heading(m.connectingLane) for m in maneuvers)
    assert headings_close(abs(normalizeAngle(headings[1] - headings[0])), math.pi)
    by_start = {m.startLane.sections[0].openDriveID: m for m in maneuvers}
    assert by_start[1].startLane.sections[0].isForward
    assert not by_start[-1].startLane.sections[0].isForward


def test_missing_rule_defaults_to_rht(tmp_path):
    path = tmp_path / "no_rule.xodr"
    path.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="6" name="no_rule" version="1.0"/>
  <road name="Default RHT" length="20.0" id="1" junction="-1">
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="20.0">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    network = Network.fromFile(path, useCache=False)
    sec = section_by_id(network, -1)
    assert sec.isForward is True


def test_unknown_rule_warns_and_assumes_rht(tmp_path):
    path = tmp_path / "bad_rule.xodr"
    path.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="6" name="bad_rule" version="1.0"/>
  <road name="Mystery" length="20.0" id="1" junction="-1" rule="FORWARD">
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="20.0">
        <line/>
      </geometry>
    </planView>
    <lanes>
      <laneSection s="0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    with pytest.warns(OpenDriveWarning, match="unrecognized rule"):
        network = Network.fromFile(path, useCache=False)
    assert section_by_id(network, -1).isForward is True
