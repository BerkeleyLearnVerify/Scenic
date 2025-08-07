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
                    x,y,z = road.st_to_xyz(float(cw.s), float(cw.t), float(cw.zOffset), float(cw.hdg))
                    print(f"Converted (s,t, zOffset) to (x, y, z): ({x:.2f}, {y:.2f}, {z:.2f})")

                except Exception as e:
                    import traceback
                    print(f"Conversion failed for crosswalk {cw.id_}")
                    traceback.print_exc()

                print(f"outlines: {cw.outlines}")

                for outline in cw.outlines:
                    for corner in outline:
                        u = corner['u']
                        v = corner['v']
                        z_local = corner['z']

                        try:
                            x,y,z = road.uv_to_xyz(float(cw.s), float(cw.t), float(cw.hdg), float(cw.pitch), float(cw.roll), u, v, z_local, float(cw.zOffset))
                            print(f"Corner (u={u:.2f}, v={v:.2f}, z={z_local:.2f}) -> (x={x:.2f}, y={y:.2f}, z={z:.2f})")
                        except Exception as e:
                            print(f"Failed to convert corner with u={u}, v={v}, z={z_local}")
                            import traceback
                            traceback.print_exc()

                print()

            total_crosswalks += len(road.crosswalks)

    print("Total crosswalks:", total_crosswalks)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Usage: python convert_st_to_xyz_test.py path_to_file.xodr")
    else:
        main(sys.argv[1])
