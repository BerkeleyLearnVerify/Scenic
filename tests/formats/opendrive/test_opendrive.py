import glob
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import pytest

from scenic.core.geometry import TriangulationError
from scenic.formats.opendrive import OpenDriveWorkspace
from scenic.formats.opendrive.xodr_parser import (
    Cubic,
    OpenDriveWarning,
    ParamCubic,
    RoadMap,
    makeCurve,
    parse_speed_to_mps,
    scenic_tags_from_type_records,
    speed_limit_from_type_records,
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


DEFAULT_PLAN_VIEW = """<planView>
      <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="20.0">
        <line/>
      </geometry>
    </planView>"""


def write_xodr(tmp_path, plan_view, road_extras=""):
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
    path = write_xodr(tmp_path, plan_view, road_extras=road_extras)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    return road_map.toScenicNetwork()


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


@pytest.mark.parametrize(
    "max_attr, unit, expected_mps",
    [
        ("25", "m/s", 25.0),
        ("90", "km/h", 90 / 3.6),
        ("55", "mph", 55 * 0.44704),
    ],
)
def test_parse_speed_to_mps_converts_units(max_attr, unit, expected_mps):
    speed_elem = ET.fromstring(f'<speed max="{max_attr}" unit="{unit}"/>')
    assert parse_speed_to_mps(speed_elem) == pytest.approx(expected_mps)


@pytest.mark.parametrize("max_attr", ["no limit", "undefined"])
def test_parse_speed_to_mps_unlimited(max_attr):
    speed_elem = ET.fromstring(f'<speed max="{max_attr}"/>')
    assert parse_speed_to_mps(speed_elem) is None


def test_parse_speed_to_mps_rejects_unknown_unit():
    speed_elem = ET.fromstring('<speed max="50" unit="knots"/>')
    with pytest.raises(ValueError, match="unsupported speed unit"):
        parse_speed_to_mps(speed_elem)


def test_speed_limit_from_type_records():
    records = [(0.0, "town", 13.89), (10.0, "motorway", 27.78)]
    assert speed_limit_from_type_records(records) == pytest.approx(13.89)


def test_speed_limit_from_type_records_empty():
    assert speed_limit_from_type_records([]) is None


def test_scenic_tags_from_type_records_maps_motorway_to_highway():
    records = [(0.0, "motorway", None)]
    assert scenic_tags_from_type_records(records) == frozenset({"highway"})


def test_scenic_tags_from_type_records_passes_through_unknown_types():
    records = [(0.0, "town", None)]
    assert scenic_tags_from_type_records(records) == frozenset({"town"})


def test_scenic_tags_from_type_records_empty():
    assert scenic_tags_from_type_records([]) == frozenset()


def test_road_type_records_parsed(tmp_path):
    path = write_xodr(
        tmp_path,
        DEFAULT_PLAN_VIEW,
        road_extras='<type s="0" type="town"><speed max="50" unit="km/h"/></type>',
    )
    road_map = RoadMap()
    road_map.parse(path)
    s, road_type, speed_mps = road_map.roads[7].type_records[0]
    assert s == pytest.approx(0.0)
    assert road_type == "town"
    assert speed_mps == pytest.approx(50 / 3.6)


def test_scenic_road_speed_limit_and_tags(tmp_path):
    network = parse_scenic_network(
        tmp_path,
        road_extras='<type s="0" type="town"><speed max="50" unit="km/h"/></type>',
    )
    road = network.roads[0]
    expected = 50 / 3.6
    assert road.speedLimit == pytest.approx(expected)
    assert road.tags == frozenset({"town"})


def test_scenic_speed_limit_propagates_to_lane_hierarchy(tmp_path):
    network = parse_scenic_network(
        tmp_path,
        road_extras='<type s="0" type="town"><speed max="36" unit="km/h"/></type>',
    )
    road = network.roads[0]
    expected = 36 / 3.6
    assert road.forwardLanes.speedLimit == pytest.approx(expected)
    assert road.sections[0].speedLimit == pytest.approx(expected)
    assert road.lanes[0].speedLimit == pytest.approx(expected)
    lane_section = road.lanes[0].sections[0]
    assert lane_section.speedLimit == pytest.approx(expected)


def test_scenic_motorway_tag_mapped_to_highway(tmp_path):
    network = parse_scenic_network(
        tmp_path,
        road_extras='<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>',
    )
    road = network.roads[0]
    assert road.tags == frozenset({"highway"})
    assert road.speedLimit == pytest.approx(120 / 3.6)


def test_scenic_road_without_type_has_no_limit_or_tags(tmp_path):
    network = parse_scenic_network(tmp_path)
    road = network.roads[0]
    assert road.speedLimit is None
    assert road.tags == frozenset()


def test_scenic_road_no_limit_speed(tmp_path):
    network = parse_scenic_network(
        tmp_path,
        road_extras='<type s="0" type="town"><speed max="no limit"/></type>',
    )
    road = network.roads[0]
    assert road.speedLimit is None
    assert road.tags == frozenset({"town"})


def test_multiple_type_segments_use_smallest_s(tmp_path):
    road_extras = """
    <type s="10" type="motorway"><speed max="120" unit="km/h"/></type>
    <type s="0" type="town"><speed max="30" unit="km/h"/></type>
    """
    with pytest.warns(
        OpenDriveWarning,
        match="road 7 has 2 type segments",
    ):
        network = parse_scenic_network(tmp_path, road_extras=road_extras)
    road = network.roads[0]
    assert road.speedLimit == pytest.approx(30 / 3.6)
    assert road.tags == frozenset({"town"})
