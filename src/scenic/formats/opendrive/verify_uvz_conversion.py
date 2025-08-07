import sys
import sys
import math

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
                #print(f"subType: {cw.subType}")
                print(f"id: {cw.id_}")

                print(f"s: {cw.s:.2f}")
                print(f"t: {cw.t:.2f}")
                print(f"zOffset: {cw.zOffset:.2f}")
                print(f"hdg: {cw.hdg:.2f}")
                print(f"orientation: {cw.orientation}")

                # print(f"length: {cw.length:.2f}")
                # print(f"width: {cw.width:.2f}")
                print(f"pitch: {cw.pitch:.8f}")
                print(f"roll: {cw.roll:.8f}")

                #print(f"markings: {len(cw.markings)}")

                try:
                    origin_x, origin_y, origin_z = road.st_to_xyz(float(cw.s), float(cw.t), float(cw.zOffset), float(cw.hdg))
                    print(f"Converted (s,t,zOffset) to (x, y, z): ({origin_x:.2f}, {origin_y:.2f}, {origin_z:.2f})")
                except Exception as e:
                    print(f"Conversion failed for crosswalk {cw.id_}")
                    import traceback
                    traceback.print_exc()
                    continue  # Skip to next crosswalk


                try:
                    test_x, test_y, test_z = road.uv_to_xyz(float(cw.s), float(cw.t), 
                    float(cw.hdg), float(cw.pitch), float(cw.roll), 0.0, 0.0, 0.0, float(cw.zOffset))

                    dx = abs(test_x - origin_x)
                    dy = abs(test_y - origin_y)
                    dz = abs(test_z - origin_z)

                    if dx < 1e-6 and dy < 1e-6 and dz < 1e-6:
                        print("Match")
                    else:
                        print("Mismatch")

                except Exception as e:
                    print(f"uv_to_xyz failed at origin check for crosswalk {cw.id_}")
                    import traceback
                    traceback.print_exc()

                #print(f"outlines: {cw.outlines}")
                print()

            total_crosswalks += len(road.crosswalks)

    print("Total crosswalks:", total_crosswalks)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Usage: python convert_uvz_to_xyz_test.py path_to_file.xodr")
    else:
        main(sys.argv[1])
