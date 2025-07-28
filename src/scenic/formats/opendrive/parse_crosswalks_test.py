import sys

from scenic.formats.opendrive import xodr_parser

def main(path):
    road_map = xodr_parser.RoadMap() #Create an instance of the RoadMap class. Empty for now
    road_map.parse(path) #Parse the .xodr file and extract things like roads, lanes, crosswalks, etc.
    total_crosswalks = 0

    for road_id, road in road_map.roads.items(): #Loop through every road 
        if road.crosswalks: 
            print(f"Road {road_id} has {len(road.crosswalks)} crosswalk(s)")

            for cw in road.crosswalks: #road.crosswalks gets appended to in xodr_parser
                #print(f"type: {cw.get('type')}")
                print(f"subType: {cw.get('subType')}")
                print(f"id: {cw.get('id')}")

                print(f"s: {cw.get('s'):.2f}")
                print(f"t: {cw.get('t'):.2f}")
                print(f"zOffset: {cw.get('zOffset'):.2f}")
                print(f"orientation: {cw.get('orientation')}")

                print(f"length: {cw.get('length'):.2f}")
                print(f"width: {cw.get('width'):.2f}")
                print(f"hdg: {cw.get('hdg'):.2f}")
                print(f"pitch: {cw.get('pitch'):.2f}")
                print(f"roll: {cw.get('roll'):.2f}")
                
                print()

            total_crosswalks += len(road.crosswalks)
    print("Total crosswalks:", total_crosswalks)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Usage: python parse_crosswalks_test.py path_to_file.xodr")
    else:
        main(sys.argv[1])
