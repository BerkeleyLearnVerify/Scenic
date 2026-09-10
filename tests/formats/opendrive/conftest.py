import xml.etree.ElementTree as ET

from scenic.formats.opendrive.xodr_parser import Junction, RoadMap

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
    tmp_path, lanes_xml, road_extras="", junction_xml="", junction_id="-1"
):
    path = tmp_path / "test.xodr"
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
  {junction_xml}
  <road name="Road 7" length="20.0" id="7" junction="{junction_id}">
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


def parse_scenic_network(
    tmp_path,
    road_extras="",
    plan_view=DEFAULT_PLAN_VIEW,
    *,
    lanes_xml=None,
    junction_xml="",
    junction_id="-1",
    junction_type=None,
    junction_name=None,
):
    if lanes_xml is None:
        path = write_xodr_with_type(tmp_path, plan_view, road_extras=road_extras)
    else:
        path = write_xodr_multi_lane(
            tmp_path,
            lanes_xml,
            road_extras=road_extras,
            junction_xml=junction_xml,
            junction_id=junction_id,
        )
    road_map = RoadMap()
    road_map.parse(path)
    road_map.calculate_geometry(num=5, calc_intersect=True)
    if junction_id != "-1":
        jid = int(junction_id)
        if jid not in road_map.junctions:
            road_map.junctions[jid] = Junction(
                jid, junction_name, junction_type or "default"
            )
    return road_map.toScenicNetwork()


def scenic_road(network, road_id=7):
    return next(road for road in network.allRoads if road.id == road_id)


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
