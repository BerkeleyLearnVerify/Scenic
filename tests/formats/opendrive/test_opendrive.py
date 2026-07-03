import glob
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import pytest

from scenic.core.geometry import TriangulationError
from scenic.domains.driving import roads as roadDomain
from scenic.formats.opendrive import OpenDriveWorkspace
from scenic.formats.opendrive.xodr_parser import (
    Cubic,
    Junction,
    OpenDriveWarning,
    ParamCubic,
    RoadMap,
    _junction_tags,
    apply_lane_speed_limits,
    assign_semantic_tags,
    effective_speed_limit_ranges,
    lane_scenic_tags,
    makeCurve,
    merge_scenic_tags,
    propagate_speed_limit,
    propagate_tags,
    speed_limit_for_s_interval,
    speed_limit_ranges_from_lane_records,
    speed_limit_ranges_from_type_records,
    speed_to_mps,
)

oldDir = os.getcwd()
os.chdir(Path("tests") / "formats" / "opendrive")
mapPath = Path("maps") / "**" / "*.xodr"
maps = glob.glob(str(mapPath))
os.chdir(oldDir)


@pytest.mark.slow
@pytest.mark.filterwarnings("ignore::scenic.formats.opendrive.OpenDriveWarning")
@pytest.mark.parametrize("path", maps)
def test_map(path, runLocally, pytestconfig):
    with runLocally():
        try:
            odw = OpenDriveWorkspace(path, n=10)
        except TriangulationError:
            pytest.skip("need better triangulation library to run this test")
        pt = odw.drivable_region.uniformPointInner()
        odw.road_direction[pt]
        if not pytestconfig.getoption("--no-graphics"):
            odw.show(plt)
            plt.show(block=False)
            plt.close()


def write_xodr(tmp_path, plan_view):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <road name="Road 7" length="20.0" id="7" junction="-1">
    {plan_view}
    <lanes>
      <laneOffset s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    return path


def test_inconsistent_planview_length_warns(tmp_path):
    path = write_xodr(
        tmp_path,
        """<planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="12.0">
        <line/>
      </geometry>
      <geometry s="10.0" x="10.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
    </planView>""",
    )

    road_map = RoadMap()

    with pytest.warns(
        OpenDriveWarning,
        match="planView of road 7 has inconsistent length",
    ):
        road_map.parse(path)

    road = road_map.roads[7]
    assert len(road.ref_line) == 2
    assert road.ref_line[0].length == pytest.approx(10.0)
    assert road.ref_line[1].length == pytest.approx(10.0)


def test_empty_planview_rejected(tmp_path):
    path = write_xodr(tmp_path, "<planView/>")

    road_map = RoadMap()

    with pytest.raises(ValueError, match="road 7 has an empty planView"):
        road_map.parse(path)


def test_planview_must_start_at_zero(tmp_path):
    path = write_xodr(
        tmp_path,
        """<planView>
      <geometry s="1.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
    </planView>""",
    )

    road_map = RoadMap()

    with pytest.raises(
        ValueError, match="reference line of road 7 does not start at s=0"
    ):
        road_map.parse(path)


def test_planview_must_be_in_order(tmp_path):
    path = write_xodr(
        tmp_path,
        """<planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
      <geometry s="-1.0" x="10.0" y="0.0" hdg="0.0" length="10.0">
        <line/>
      </geometry>
    </planView>""",
    )

    road_map = RoadMap()

    with pytest.raises(ValueError, match="planView of road 7 is not in order"):
        road_map.parse(path)


def test_make_curve_poly3():
    curve_elem = ET.fromstring('<poly3 a="0.0" b="1.0" c="0.0" d="0.0"/>')

    curve, susp = makeCurve(0.0, 0.0, 0.0, 10.0, curve_elem)

    assert isinstance(curve, Cubic)
    assert curve.length == pytest.approx(10.0)
    assert not susp


def test_make_curve_param_poly3():
    curve_elem = ET.fromstring(
        '<paramPoly3 aU="0.0" bU="1.0" cU="0.0" dU="0.0" '
        'aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="normalized"/>'
    )

    curve, susp = makeCurve(0.0, 0.0, 0.0, 10.0, curve_elem)

    assert isinstance(curve, ParamCubic)
    assert curve.length == pytest.approx(10.0)
    assert not susp


def test_make_curve_param_poly3_rejects_arc_length():
    curve_elem = ET.fromstring(
        '<paramPoly3 aU="0.0" bU="1.0" cU="0.0" dU="0.0" '
        'aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="arcLength"/>'
    )

    with pytest.raises(NotImplementedError, match="unsupported pRange for paramPoly3"):
        makeCurve(0.0, 0.0, 0.0, 10.0, curve_elem)


def test_make_curve_rejects_unknown_geometry_type():
    curve_elem = ET.fromstring("<unknown/>")

    with pytest.raises(NotImplementedError, match="unhandled OpenDRIVE geometry type"):
        makeCurve(0.0, 0.0, 0.0, 10.0, curve_elem)


DEFAULT_PLAN_VIEW = """<planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0">
        <line/>
      </geometry>
    </planView>"""


def write_xodr_with_type(tmp_path, plan_view=DEFAULT_PLAN_VIEW, road_extras=""):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <road name="Road 7" length="20.0" id="7" junction="-1">
    {road_extras}
    {plan_view}
    <lanes>
      <laneOffset s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
      <laneSection s="0.0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <right>
          <lane id="-1" type="driving" level="false">
            <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    return path


def parse_scenic_network(tmp_path, road_extras="", plan_view=DEFAULT_PLAN_VIEW):
    path = write_xodr_with_type(tmp_path, plan_view, road_extras=road_extras)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    return road_map.toScenicNetwork()


def type_tags_from_road_extras(road_extras):
    root = ET.fromstring(f"<root>{road_extras}</root>")
    return frozenset(elem.get("type") for elem in root.iter("type") if elem.get("type"))


def assert_road_level_tags_propagated(road):
    """Road-level tags should match lane groups and road sections only."""
    expected = road.tags
    if road.forwardLanes is not None:
        assert road.forwardLanes.tags == expected
    if road.backwardLanes is not None:
        assert road.backwardLanes.tags == expected
    for section in road.sections:
        assert section.tags == expected


def write_xodr_multi_lane(tmp_path, lanes_xml, road_extras="", junction_xml=""):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  {junction_xml}
  <road name="Road 7" length="20.0" id="7" junction="-1">
    {road_extras}
    {DEFAULT_PLAN_VIEW}
    <lanes>
      <laneOffset s="0" a="0" b="0" c="0" d="0"/>
      <laneSection s="0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <right>
{lanes_xml}
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    return path


def _linear_element_stub(cls):
    element = object.__new__(cls)
    element.speedLimit = None
    element.tags = frozenset()
    return element


@pytest.mark.parametrize(
    "cls",
    [
        roadDomain.RoadSection,
        roadDomain.LaneGroup,
    ],
)
def test_propagate_tags(cls):
    element = _linear_element_stub(cls)
    tags = frozenset({"from_map"})
    propagate_tags(tags, [element, "ignored"])
    assert element.tags == tags


def test_propagate_tags_skips_lane_elements():
    lane_section = _linear_element_stub(roadDomain.LaneSection)
    lane = _linear_element_stub(roadDomain.Lane)
    tags = frozenset({"from_map"})
    propagate_tags(tags, [lane_section, lane])
    assert lane_section.tags == frozenset()
    assert lane.tags == frozenset()


def test_propagate_tags_skips_empty():
    element = _linear_element_stub(roadDomain.LaneSection)
    propagate_tags(frozenset(), [element])
    assert element.tags == frozenset()


def test_propagate_tags_ignores_other_types():
    element = _linear_element_stub(roadDomain.RoadSection)
    tags = frozenset({"from_map"})
    propagate_tags(tags, [object(), element])
    assert element.tags == tags


def test_propagate_tags_does_not_overwrite_existing():
    element = _linear_element_stub(roadDomain.Lane)
    existing = frozenset({"existing"})
    element.tags = existing
    propagate_tags(frozenset({"from_map"}), [element])
    assert element.tags == existing


def test_map_tags_propagate_to_lane_hierarchy(tmp_path):
    road_extras = '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
    network = parse_scenic_network(tmp_path, road_extras=road_extras)
    road = network.roads[0]
    assert road.tags == type_tags_from_road_extras(road_extras)
    assert_road_level_tags_propagated(road)
    assert road.lanes[0].tags == frozenset({"driving"})
    assert road.lanes[0].sections[0].tags == road.lanes[0].tags


def test_map_tags_from_all_type_segments_propagate(tmp_path):
    road_extras = (
        '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
        '<type s="10" type="town"><speed max="50" unit="km/h"/></type>'
    )
    network = parse_scenic_network(tmp_path, road_extras=road_extras)
    road = network.roads[0]
    assert road.tags == type_tags_from_road_extras(road_extras)
    assert_road_level_tags_propagated(road)
    assert road.lanes[0].tags == frozenset({"driving"})


def test_lane_type_tags_are_lane_specific(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
          <lane id="-2" type="onRamp" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>"""
    road_extras = '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
    path = write_xodr_multi_lane(tmp_path, lanes_xml, road_extras=road_extras)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    road = road_map.toScenicNetwork().roads[0]

    tags_by_od_id = {
        section.openDriveID: section.tags for section in road.sections[0].lanes
    }
    assert tags_by_od_id[-1] == frozenset({"driving"})
    assert tags_by_od_id[-2] == frozenset({"onRamp"})
    assert "onRamp" not in tags_by_od_id[-1]
    assert "driving" not in tags_by_od_id[-2]
    assert road.tags == frozenset({"motorway"})


def test_junction_ramp_tags_apply_only_to_ramp_lanes(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
          <lane id="-2" type="onRamp" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>"""
    road_extras = '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
    path = write_xodr_multi_lane(tmp_path, lanes_xml, road_extras=road_extras)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    internal_road = road_map.roads[7]
    internal_road.extra_tags = frozenset({"onramp"})
    road, _ = internal_road.toScenicRoad(road_map.tolerance)

    assert road.tags == frozenset({"motorway", "onramp"})
    tags_by_od_id = {
        section.openDriveID: section.tags for section in road.sections[0].lanes
    }
    assert tags_by_od_id[-1] == frozenset({"driving"})
    assert tags_by_od_id[-2] == frozenset({"onRamp"})
    assert "onramp" not in tags_by_od_id[-1]
    assert "onramp" not in tags_by_od_id[-2]
    assert "motorway" not in tags_by_od_id[-1]


def test_lane_scenic_tags_uses_open_drive_values_as_is():
    assert lane_scenic_tags("onRamp") == frozenset({"onRamp"})
    assert lane_scenic_tags("driving") == frozenset({"driving"})
    assert lane_scenic_tags(None) == frozenset()


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
        (0.0, 20.0, 45.0),
        (20.0, 30.0, 30.0),
    ]


def test_speed_limit_for_s_interval_single_and_multiple():
    ranges = [
        (0.0, 7.0, 45.0),
        (7.0, 10.0, 30.0),
        (10.0, 30.0, 45.0),
    ]
    speed, overlapping = speed_limit_for_s_interval(ranges, 10.0, 20.0)
    assert speed == pytest.approx(45.0)
    assert overlapping == frozenset({45.0})

    speed, overlapping = speed_limit_for_s_interval(ranges, 0.0, 10.0)
    assert speed == pytest.approx(30.0)
    assert overlapping == frozenset({45.0, 30.0})


def test_speed_limit_ranges_from_lane_records():
    records = [(0.0, 20.0), (5.0, 30.0)]
    assert speed_limit_ranges_from_lane_records(records, 10.0) == [
        (0.0, 5.0, 20.0),
        (5.0, 10.0, 30.0),
    ]


def test_speed_limit_ranges_from_lane_records_carry_forward():
    records = [(0.0, 20.0), (5.0, None), (8.0, 30.0)]
    assert speed_limit_ranges_from_lane_records(records, 10.0) == [
        (0.0, 8.0, 20.0),
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
    assert lane.speed_limit == pytest.approx(20.0)
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
    path = write_xodr_lane_speeds(tmp_path, lanes_xml)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    lane_section = road_map.toScenicNetwork().roads[0].sections[0].lanes[0]

    assert lane_section.speedLimit == pytest.approx(20.0)
    assert lane_section.speedLimitRanges == (
        (0.0, 5.0, 20.0),
        (5.0, 20.0, 30.0),
    )
    assert lane_section.speedLimitAt(2.0) == pytest.approx(20.0)
    assert lane_section.speedLimitAt(7.0) == pytest.approx(30.0)


def test_propagate_speed_limit_sets_section_specific_values():
    road_section = _linear_element_stub(roadDomain.RoadSection)
    lane_section = _linear_element_stub(roadDomain.LaneSection)
    propagate_speed_limit(30.0, [road_section, lane_section])
    assert road_section.speedLimit == pytest.approx(30.0)
    assert lane_section.speedLimit == pytest.approx(30.0)

    other_section = _linear_element_stub(roadDomain.RoadSection)
    other_lane = _linear_element_stub(roadDomain.LaneSection)
    propagate_speed_limit(45.0, [other_section, other_lane])
    assert other_section.speedLimit == pytest.approx(45.0)
    assert other_lane.speedLimit == pytest.approx(45.0)


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
        speed_limit_for_s_interval(speed_ranges, s_start, s_end)
        for s_start, s_end in section_intervals
    ]

    assert section_speeds[0] == (pytest.approx(30.0), frozenset({45.0, 30.0}))
    assert section_speeds[1] == (pytest.approx(45.0), frozenset({45.0}))
    assert section_speeds[2] == (pytest.approx(45.0), frozenset({45.0}))
    assert min(speed for speed, _ in section_speeds) == pytest.approx(30.0)


def write_xodr_lane_speeds(tmp_path, lanes_xml, road_extras=""):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <road name="Road 7" length="20.0" id="7" junction="-1">
    {road_extras}
    {DEFAULT_PLAN_VIEW}
    <lanes>
      <laneOffset s="0" a="0" b="0" c="0" d="0"/>
      <laneSection s="0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <right>
{lanes_xml}
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    return path


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
    path = write_xodr_lane_speeds(tmp_path, lanes_xml, road_extras=road_extras)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    road = road_map.toScenicNetwork().roads[0]

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


def test_merge_scenic_tags_combines_dedups_and_ignores_empty():
    assert merge_scenic_tags(frozenset({"a"}), frozenset({"a", "b"})) == frozenset(
        {"a", "b"}
    )
    assert merge_scenic_tags(frozenset(), None, frozenset({"x"})) == frozenset({"x"})
    assert merge_scenic_tags() == frozenset()


def test_junction_tags_from_type_and_name():
    assert _junction_tags(Junction(1, "Roundabout A", "roundabout")) == frozenset(
        {"roundabout", "Roundabout A"}
    )
    # The default junction type carries no semantic meaning and is dropped.
    assert _junction_tags(Junction(2, None, "default")) == frozenset()
    # type_ defaults to "default", so only the name remains.
    assert _junction_tags(Junction(3, "J3")) == frozenset({"J3"})


class _FakeRoad:
    def __init__(self, junction):
        self.junction = junction
        self.extra_tags = frozenset()


class _FakeRoadMap:
    def __init__(self, roads, junctions):
        self.roads = roads
        self.junctions = junctions


def test_assign_semantic_tags_tags_only_roads_in_junction():
    road_map = _FakeRoadMap(
        roads={10: _FakeRoad(junction=5), 11: _FakeRoad(junction=None)},
        junctions={5: Junction(5, "J5", "roundabout")},
    )
    assign_semantic_tags(road_map)
    assert road_map.roads[10].extra_tags == frozenset({"roundabout", "J5"})
    assert road_map.roads[11].extra_tags == frozenset()


def test_apply_lane_speed_limits_ignores_non_lane_elements():
    lane = _linear_element_stub(roadDomain.Lane)
    lane.sections = [
        _linear_element_stub(roadDomain.LaneSection),
        _linear_element_stub(roadDomain.LaneSection),
    ]
    lane.sections[0].speedLimit = 10.0
    lane.sections[1].speedLimit = 25.0
    section = _linear_element_stub(roadDomain.RoadSection)
    apply_lane_speed_limits([lane, section, "ignored"])
    assert lane.speedLimit == pytest.approx(25.0)
    assert section.speedLimit is None


def test_section_spanning_multiple_speeds_warns(tmp_path):
    road_extras = (
        '<type s="0" type="town"><speed max="50" unit="km/h"/></type>'
        '<type s="10" type="town"><speed max="30" unit="km/h"/></type>'
    )
    # The single 20 m lane section [0,20) straddles both the 50 and 30 km/h limits.
    path = write_xodr_lane_speeds(
        tmp_path,
        """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>""",
        road_extras=road_extras,
    )
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    with pytest.warns(OpenDriveWarning, match="spans multiple speed limits"):
        road_map.toScenicNetwork()
