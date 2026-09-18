from scenic.formats.opendrive.xodr_parser import Junction, RoadMap, assign_semantic_tags

from .conftest import (
    assert_road_tags_propagated_to_groups,
    parse_scenic_network,
    scenic_road,
    type_tags_from_road_extras,
    write_xodr_junction,
    write_xodr_lane_sections,
)


def test_map_tags_propagate_to_lane_hierarchy(tmp_path):
    road_extras = '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
    network = parse_scenic_network(tmp_path, road_extras=road_extras)
    road = network.roads[0]
    assert road.tags == type_tags_from_road_extras(road_extras)
    assert_road_tags_propagated_to_groups(road)
    assert road.sections[0].tags == road.tags
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
    assert_road_tags_propagated_to_groups(road)
    assert road.sections[0].tags == road.tags
    assert road.lanes[0].tags == frozenset({"driving"})


def test_road_section_tags_follow_type_segments(tmp_path):
    lane_sections_xml = """      <laneSection s="0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><successor id="-1"/></link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
        </right>
      </laneSection>
      <laneSection s="10">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><predecessor id="-1"/></link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
        </right>
      </laneSection>"""
    road_extras = '<type s="0" type="motorway"/><type s="10" type="town"/>'
    path = write_xodr_lane_sections(tmp_path, lane_sections_xml, road_extras=road_extras)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    road = scenic_road(road_map.toScenicNetwork())

    assert road.tags == frozenset({"motorway", "town"})
    assert road.sections[0].tags == road.tags
    assert road.sections[1].tags == road.tags


def test_lane_type_tags_are_lane_specific(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
          <lane id="-2" type="onRamp" level="false">
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>"""
    road_extras = '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
    network = parse_scenic_network(tmp_path, lanes_xml=lanes_xml, road_extras=road_extras)
    road = scenic_road(network)

    tags_by_od_id = {
        section.openDriveID: section.tags for section in road.sections[0].lanes
    }
    assert tags_by_od_id[-1] == frozenset({"driving"})
    assert tags_by_od_id[-2] == frozenset({"onRamp"})
    assert "onRamp" not in tags_by_od_id[-1]
    assert "driving" not in tags_by_od_id[-2]
    assert road.tags == frozenset({"motorway"})


def test_junction_type_tags_apply_only_to_road_level(tmp_path):
    lanes_xml = """          <lane id="-1" type="driving" level="false">
            <link>
              <predecessor id="-1"/>
              <successor id="-1"/>
            </link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
          <lane id="-2" type="onRamp" level="false">
            <link>
              <predecessor id="-2"/>
              <successor id="-2"/>
            </link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>"""
    path = write_xodr_junction(tmp_path, lanes_xml)
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    network = road_map.toScenicNetwork()
    road = scenic_road(network)

    assert road.tags == frozenset({"direct", "motorway"})
    tags_by_od_id = {
        section.openDriveID: section.tags for section in road.sections[0].lanes
    }
    assert tags_by_od_id[-1] == frozenset({"driving"})
    assert tags_by_od_id[-2] == frozenset({"onRamp"})
    assert "direct" not in tags_by_od_id[-1]
    assert "direct" not in tags_by_od_id[-2]
    assert "motorway" not in tags_by_od_id[-1]


def test_junction_tags_from_type():
    assert Junction(1, "Direct junction", "direct").tags == frozenset({"direct"})
    # The default junction type carries no semantic meaning and is dropped.
    assert Junction(2, None, "default").tags == frozenset()
    # type_ defaults to "default"; junction names are not semantic tags.
    assert Junction(3, "J3").tags == frozenset()
    # Unknown type strings are still preserved as uninterpreted tags.
    assert Junction(4, "J4", "roundabout").tags == frozenset({"roundabout"})


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
        junctions={5: Junction(5, "J5", "virtual")},
    )
    assign_semantic_tags(road_map)
    assert road_map.roads[10].extra_tags == frozenset({"virtual"})
    assert road_map.roads[11].extra_tags == frozenset()
