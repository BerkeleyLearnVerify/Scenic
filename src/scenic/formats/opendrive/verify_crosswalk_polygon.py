import sys
import math
import traceback

from scenic.formats.opendrive import xodr_parser


def test_crosswalk_polygon(road_id, road):
    if road.crosswalks: 
        print(f"Road {road_id} has {len(road.crosswalks)} crosswalk(s)")

        for cw in road.crosswalks:
            try:
                road.construct_crosswalk_polys(cw, 0.05)
            except Exception as e:
                print(f"Failed: {cw.id_}")
                traceback.print_exc()

            print()


def main(path):
    road_map = xodr_parser.RoadMap()
    road_map.parse(path)

    for road_id, road in road_map.roads.items():
        test_crosswalk_polygon(road_id, road)

        

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Usage: python verify_crosswalk_polygon.py path_to_file.xodr")
    else:
        main(sys.argv[1])
