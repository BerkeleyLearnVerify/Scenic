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
                #print(f"type: {cw.type_}")
                print(f"subType: {cw.subType}")
                print(f"id: {cw.id_}")

                print(f"s: {cw.s:.2f}")
                print(f"t: {cw.t:.2f}")
                print(f"zOffset: {cw.zOffset:.2f}")
                print(f"orientation: {cw.orientation}")

                print(f"length: {cw.length:.2f}")
                print(f"width: {cw.width:.2f}")
                print(f"hdg: {cw.hdg:.2f}")
                print(f"pitch: {cw.pitch:.8f}")
                print(f"roll: {cw.roll:.8f}")

                print(f"outlines: {cw.outlines}")
                print(f"markings: {len(cw.markings)}")

                print()

            total_crosswalks += len(road.crosswalks)

    print("Total crosswalks:", total_crosswalks)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Usage: python parse_crosswalks_test.py path_to_file.xodr")
    else:
        main(sys.argv[1])
