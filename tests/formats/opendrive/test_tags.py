from scenic.formats.opendrive.xodr_parser import Junction

from .conftest import (
    TWO_LANE_SECTIONS,
    assert_road_tags_propagated_to_groups,
    lane_xml,
    parse_scenic_network,
    scenic_road,
    type_tags_from_road_extras,
)


def test_map_tags_propagate_to_lane_hierarchy(tmp_path):
    road_extras = '<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>'
    road = parse_scenic_network(tmp_path, road_extras=road_extras).roads[0]
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
    road = parse_scenic_network(tmp_path, road_extras=road_extras).roads[0]
    assert road.tags == type_tags_from_road_extras(road_extras)
    assert_road_tags_propagated_to_groups(road)
    assert road.sections[0].tags == road.tags
    assert road.lanes[0].tags == frozenset({"driving"})


def test_road_section_tags_follow_type_segments(tmp_path):
    road = scenic_road(
        parse_scenic_network(
            tmp_path,
            lane_sections_xml=TWO_LANE_SECTIONS,
            road_extras='<type s="0" type="motorway"/><type s="10" type="town"/>',
        )
    )
    assert road.tags == frozenset({"motorway", "town"})
    assert road.sections[0].tags == road.tags
    assert road.sections[1].tags == road.tags


def test_lane_type_tags_are_lane_specific(tmp_path):
    road = scenic_road(
        parse_scenic_network(
            tmp_path,
            lanes_xml="\n".join((lane_xml(-1), lane_xml(-2, type_="onRamp"))),
            road_extras='<type s="0" type="motorway"><speed max="120" unit="km/h"/></type>',
        )
    )
    tags = {section.openDriveID: section.tags for section in road.sections[0].lanes}
    assert tags[-1] == frozenset({"driving"})
    assert tags[-2] == frozenset({"onRamp"})
    assert road.tags == frozenset({"motorway"})


def test_junction_type_tags_apply_only_to_connecting_road(tmp_path):
    network = parse_scenic_network(
        tmp_path,
        junction_lanes_xml="\n".join(
            (
                lane_xml(-1, pred=-1, succ=-1),
                lane_xml(-2, type_="onRamp", pred=-2, succ=-2),
            )
        ),
    )
    connecting = scenic_road(network)
    incoming = scenic_road(network, road_id=6)
    tags = {section.openDriveID: section.tags for section in connecting.sections[0].lanes}

    assert connecting.tags == frozenset({"direct", "motorway"})
    assert "direct" not in incoming.tags
    assert incoming.tags == frozenset()
    assert tags[-1] == frozenset({"driving"})
    assert tags[-2] == frozenset({"onRamp"})
    assert "direct" not in tags[-1]
    assert "motorway" not in tags[-1]


def test_junction_tags_from_type():
    assert Junction(1, "Direct junction", "direct").tags == frozenset({"direct"})
    assert Junction(2, None, "default").tags == frozenset()
    assert Junction(3, "J3").tags == frozenset()
    assert Junction(4, "J4", "roundabout").tags == frozenset({"roundabout"})
