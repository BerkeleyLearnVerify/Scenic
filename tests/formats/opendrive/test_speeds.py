import xml.etree.ElementTree as ET

import pytest

from scenic.formats.opendrive.xodr_parser import (
    OpenDriveWarning,
    RoadMap,
    effective_speed_limit_ranges,
    speed_limits_for_s_interval,
    speed_limit_ranges_from_lane_records,
    speed_limit_ranges_from_type_records,
    speed_to_mps,
)

from .conftest import parse_scenic_network, scenic_road, write_xodr_lane_speeds


def test_speed_limit_ranges_from_type_records():
    records = [
        (0.0, "town", 45.0),
        (7.0, "town", 30.0),
        (10.0, "town", 45.0),
        (15.0, "town", 45.0),
    ]
    assert speed_limit_ranges_from_type_records(records, 30.0) == [
        (0.0, 7.0, 45.0),
        (7.0, 10.0, 30.0),
        (10.0, 30.0, 45.0),
    ]


def test_speed_limit_ranges_carry_forward_missing_speed():
    records = [
        (0.0, "town", 45.0),
        (10.0, "motorway", None),
        (20.0, "motorway", 30.0),
    ]
    assert speed_limit_ranges_from_type_records(records, 30.0) == [
        (0.0, 10.0, 45.0),
        (10.0, 20.0, None),
        (20.0, 30.0, 30.0),
    ]


def test_speed_limits_for_s_interval_single_and_multiple():
    ranges = [
        (0.0, 7.0, 45.0),
        (7.0, 10.0, 30.0),
        (10.0, 30.0, 45.0),
    ]
    overlapping = speed_limits_for_s_interval(ranges, 10.0, 20.0)
    assert overlapping == frozenset({45.0})
    assert min(overlapping) == pytest.approx(45.0)

    overlapping = speed_limits_for_s_interval(ranges, 0.0, 10.0)
    assert overlapping == frozenset({45.0, 30.0})
    assert min(overlapping) == pytest.approx(30.0)


def test_speed_limit_ranges_from_lane_records():
    records = [(0.0, 20.0), (5.0, 30.0)]
    assert speed_limit_ranges_from_lane_records(records, 10.0) == [
        (0.0, 5.0, 20.0),
        (5.0, 10.0, 30.0),
    ]


def test_speed_limit_ranges_from_lane_records_carry_forward():
    records = [(0.0, 20.0), (5.0, None), (8.0, 30.0)]
    assert speed_limit_ranges_from_lane_records(records, 10.0) == [
        (0.0, 5.0, 20.0),
        (5.0, 8.0, None),
        (8.0, 10.0, 30.0),
    ]


def test_effective_speed_limit_ranges_lane_overrides_road():
    road_ranges = [(0.0, 10.0, 50.0)]
    lane_ranges = [(0.0, 10.0, 80.0)]
    assert effective_speed_limit_ranges(road_ranges, lane_ranges, 0.0, 10.0) == [
        (0.0, 10.0, 80.0),
    ]


def test_effective_speed_limit_ranges_multiple_lane_speeds():
    road_ranges = [(0.0, 10.0, 50.0)]
    lane_ranges = [(0.0, 5.0, 20.0), (5.0, 10.0, 30.0)]
    assert effective_speed_limit_ranges(road_ranges, lane_ranges, 0.0, 10.0) == [
        (0.0, 5.0, 20.0),
        (5.0, 10.0, 30.0),
    ]


def test_effective_speed_limit_ranges_lane_speed_below_road():
    road_ranges = [(0.0, 10.0, 50.0)]
    lane_ranges = [(0.0, 10.0, 30.0)]
    assert effective_speed_limit_ranges(road_ranges, lane_ranges, 0.0, 10.0) == [
        (0.0, 10.0, 30.0),
    ]


def test_effective_speed_limit_ranges_road_only_uses_road_ranges():
    road_ranges = [(0.0, 5.0, 50.0), (5.0, 10.0, 30.0)]
    assert effective_speed_limit_ranges(road_ranges, [], 0.0, 10.0) == [
        (0.0, 5.0, 50.0),
        (5.0, 10.0, 30.0),
    ]


def test_parse_lane_speed_records(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <speed sOffset="0" max="20" unit="m/s"/>
            <speed sOffset="5" max="30" unit="m/s"/>
          </lane>"""
    path = write_xodr_lane_speeds(tmp_path, lanes_xml)
    road_map = RoadMap()
    road_map.parse(path)
    lane = road_map.roads[7].lane_secs[0].get_lane(-1)
    assert lane.speed_records == [(0.0, 20.0), (5.0, 30.0)]
    assert speed_limit_ranges_from_lane_records(lane.speed_records, 10.0) == [
        (0.0, 5.0, 20.0),
        (5.0, 10.0, 30.0),
    ]


def test_lane_speed_ranges_on_scenic_lane_section(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <speed sOffset="0" max="20" unit="m/s"/>
            <speed sOffset="5" max="30" unit="m/s"/>
          </lane>"""
    network = parse_scenic_network(tmp_path, lanes_xml=lanes_xml)
    lane_section = scenic_road(network).sections[0].lanes[0]

    assert lane_section.speedLimit == pytest.approx(20.0)
    assert lane_section.speedLimitRanges == (
        (0.0, 5.0, 20.0),
        (5.0, 20.0, 30.0),
    )
    assert lane_section.speedLimitAt(2.0) == pytest.approx(20.0)
    assert lane_section.speedLimitAt(7.0) == pytest.approx(30.0)

    # Network.speedLimitAt projects onto the section centerline and looks up
    # the matching range.
    slow_point = lane_section.centerline.pointAlongBy(2.0)
    fast_point = lane_section.centerline.pointAlongBy(7.0)
    assert network.speedLimitAt(slow_point) == pytest.approx(20.0)
    assert network.speedLimitAt(fast_point) == pytest.approx(30.0)
    assert network.speedLimitAt((1000.0, 1000.0)) is None


def test_network_speed_limit_at_uniform_section(tmp_path):
    road_extras = '<type s="0" type="town"><speed max="50" unit="km/h"/></type>'
    network = parse_scenic_network(tmp_path, road_extras=road_extras)
    road = scenic_road(network)
    point = road.lanes[0].sections[0].centerline.pointAlongBy(5.0)
    assert network.speedLimitAt(point) == pytest.approx(50 / 3.6)


def test_no_limit_speed_sets_ranges_with_none(tmp_path):
    road_extras = (
        '<type s="0" type="motorway"><speed max="no limit" unit="km/h"/></type>'
    )
    network = parse_scenic_network(tmp_path, road_extras=road_extras)
    lane_section = scenic_road(network).lanes[0].sections[0]
    assert lane_section.speedLimit is None
    assert lane_section.speedLimitRanges == ((0.0, 20.0, None),)
    point = lane_section.centerline.pointAlongBy(5.0)
    assert network.speedLimitAt(point) is None


def test_road_speed_limits_vary_by_lane_section():
    speed_ranges = speed_limit_ranges_from_type_records(
        [
            (0.0, "town", 45.0),
            (7.0, "town", 30.0),
            (10.0, "town", 45.0),
            (15.0, "town", 45.0),
            (20.0, "town", 45.0),
            (30.0, "town", 45.0),
        ],
        30.0,
    )
    section_intervals = [(0.0, 10.0), (10.0, 20.0), (20.0, 30.0)]
    section_speeds = [
        speed_limits_for_s_interval(speed_ranges, s_start, s_end)
        for s_start, s_end in section_intervals
    ]

    assert section_speeds[0] == frozenset({45.0, 30.0})
    assert section_speeds[1] == frozenset({45.0})
    assert section_speeds[2] == frozenset({45.0})
    assert min(min(speeds) for speeds in section_speeds) == pytest.approx(30.0)


def test_lane_speed_limit_overrides_when_higher(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
          <lane id="-2" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <speed sOffset="0" max="80" unit="km/h"/>
          </lane>
          <lane id="-3" type="onRamp" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <speed sOffset="0" max="40" unit="km/h"/>
          </lane>"""
    road_extras = '<type s="0" type="town"><speed max="50" unit="km/h"/></type>'
    network = parse_scenic_network(
        tmp_path, lanes_xml=lanes_xml, road_extras=road_extras
    )
    road = scenic_road(network)

    road_limit = 50 / 3.6
    fast_limit = 80 / 3.6
    assert road.speedLimit == pytest.approx(road_limit)
    limits_by_od_id = {
        section.openDriveID: section.speedLimit for section in road.sections[0].lanes
    }
    assert limits_by_od_id[-1] == pytest.approx(road_limit)
    assert limits_by_od_id[-2] == pytest.approx(fast_limit)
    assert limits_by_od_id[-3] == pytest.approx(40 / 3.6)


def _speed_elem(max_value, unit=None):
    attrs = f'max="{max_value}"'
    if unit is not None:
        attrs += f' unit="{unit}"'
    return ET.fromstring(f"<speed {attrs}/>")


def test_speed_to_mps_unit_conversions():
    assert speed_to_mps(_speed_elem(36, "km/h")) == pytest.approx(10.0)
    assert speed_to_mps(_speed_elem(10, "m/s")) == pytest.approx(10.0)
    assert speed_to_mps(_speed_elem(100, "mph")) == pytest.approx(44.704)
    # A missing unit defaults to m/s.
    assert speed_to_mps(_speed_elem(15)) == pytest.approx(15.0)


def test_speed_to_mps_unlimited_returns_none():
    assert speed_to_mps(_speed_elem("no limit", "km/h")) is None
    assert speed_to_mps(_speed_elem("undefined")) is None


def test_speed_to_mps_rejects_unknown_unit():
    with pytest.raises(ValueError, match="unsupported speed unit"):
        speed_to_mps(_speed_elem(50, "furlongs/fortnight"))


def test_section_spanning_multiple_speeds_warns(tmp_path):
    road_extras = (
        '<type s="0" type="town"><speed max="50" unit="km/h"/></type>'
        '<type s="10" type="town"><speed max="30" unit="km/h"/></type>'
    )
    # The single 20 m lane section [0,20) straddles both the 50 and 30 km/h limits.
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>"""
    with pytest.warns(OpenDriveWarning, match="spans multiple speed limits"):
        parse_scenic_network(
            tmp_path, lanes_xml=lanes_xml, road_extras=road_extras
        )
