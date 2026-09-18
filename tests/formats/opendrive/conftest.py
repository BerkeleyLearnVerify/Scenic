import xml.etree.ElementTree as ET

from scenic.formats.opendrive.xodr_parser import RoadMap

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


def write_xodr_multi_lane(
    tmp_path,
    lanes_xml,
    road_extras="",
    lane_side="right",
    plan_view=DEFAULT_PLAN_VIEW,
):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <road name="Road 7" length="20.0" id="7" junction="-1">
    {road_extras}
    {plan_view}
    <lanes>
      <laneOffset s="0" a="0" b="0" c="0" d="0"/>
      <laneSection s="0">
        <center>
          <lane id="0" type="none" level="false"/>
        </center>
        <{lane_side}>
{lanes_xml}
        </{lane_side}>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    return path


def write_xodr_lane_speeds(tmp_path, lanes_xml, road_extras="", lane_side="right"):
    return write_xodr_multi_lane(
        tmp_path, lanes_xml, road_extras=road_extras, lane_side=lane_side
    )


def write_xodr_lane_sections(tmp_path, lane_sections_xml, road_extras=""):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  <road name="Road 7" length="20.0" id="7" junction="-1">
    {road_extras}
    {DEFAULT_PLAN_VIEW}
    <lanes>
      <laneOffset s="0" a="0" b="0" c="0" d="0"/>
{lane_sections_xml}
    </lanes>
  </road>
</OpenDRIVE>
"""
    )
    return path


def write_xodr_junction(tmp_path, connecting_lanes_xml):
    path = tmp_path / "test.xodr"
    support_lanes_xml = """<lane id="-1" type="driving" level="false">
            <link>
              <predecessor id="-1"/>
              <successor id="-1"/>
            </link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>
          <lane id="-2" type="driving" level="false">
            <link>
              <predecessor id="-2"/>
              <successor id="-2"/>
            </link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
          </lane>"""

    def road_xml(id_, junction, x, lanes_xml, link_xml, road_extras=""):
        return f"""<road name="Road {id_}" length="20" id="{id_}" junction="{junction}">
    {link_xml}
    {road_extras}
    <planView>
      <geometry s="0" x="{x}" y="0" hdg="0" length="20"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0">
        <center><lane id="0" type="none" level="false"/></center>
        <right>
          {lanes_xml}
        </right>
      </laneSection>
    </lanes>
  </road>"""

    incoming_link = '<link><successor elementType="junction" elementId="5"/></link>'
    connecting_link = """<link>
      <predecessor elementType="road" elementId="6" contactPoint="end"/>
      <successor elementType="road" elementId="8" contactPoint="start"/>
    </link>"""
    outgoing_link = '<link><predecessor elementType="junction" elementId="5"/></link>'
    connecting_road_xml = road_xml(
        7,
        5,
        0,
        connecting_lanes_xml,
        connecting_link,
        '<type s="0" type="motorway"/>',
    )
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  {road_xml(6, -1, -20, support_lanes_xml, incoming_link)}
  {connecting_road_xml}
  {road_xml(8, -1, 20, support_lanes_xml, outgoing_link)}
  <junction name="J5" id="5" type="direct">
    <connection id="0" incomingRoad="6" connectingRoad="7" contactPoint="start">
      <laneLink from="-1" to="-1"/>
      <laneLink from="-2" to="-2"/>
    </connection>
  </junction>
</OpenDRIVE>
"""
    )
    return path


def parse_scenic_network(
    tmp_path,
    road_extras="",
    plan_view=DEFAULT_PLAN_VIEW,
    *,
    lanes_xml=None,
    lane_side="right",
):
    if lanes_xml is None:
        path = write_xodr_with_type(tmp_path, plan_view, road_extras=road_extras)
    else:
        path = write_xodr_multi_lane(
            tmp_path,
            lanes_xml,
            road_extras=road_extras,
            lane_side=lane_side,
            plan_view=plan_view,
        )
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    return road_map.toScenicNetwork()


def scenic_road(network, road_id=7):
    return next(road for road in network.allRoads if road.id == road_id)


def type_tags_from_road_extras(road_extras):
    root = ET.fromstring(f"<root>{road_extras}</root>")
    return frozenset(elem.get("type") for elem in root.iter("type") if elem.get("type"))


def assert_road_tags_propagated_to_groups(road):
    """Road-level tags should propagate to the lane groups spanning the road."""
    expected = road.tags
    if road.forwardLanes is not None:
        assert road.forwardLanes.tags == expected
    if road.backwardLanes is not None:
        assert road.backwardLanes.tags == expected
