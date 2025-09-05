import sys
import math
import traceback

from scenic.formats.opendrive import xodr_parser


def verify_crosswalks(road_id, road):
    if road.crosswalks: 
        print(f"Road {road_id} has {len(road.crosswalks)} crosswalk(s)")

        for cw in road.crosswalks:
            print(f"type: {cw.type_}")
            print(f"subType: {cw.subtype}")
            print(f"id: {cw.id_}")

            #print(f"s: {cw.s:.2f}")
            #print(f"t: {cw.t:.2f}")
            #print(f"zOffset: {cw.zOffset:.2f}")
            print(f"hdg: {cw.hdg:.2f}")
            print(f"orientation: {cw.orientation}")

            print(f"length: {cw.length:.2f}")
            print(f"width: {cw.width:.2f}")
            print(f"pitch: {cw.pitch:.8f}")
            print(f"roll: {cw.roll:.8f}")

            print(f"outlines: {cw.outlines}")

            print()

            #Test st
            try:
                x,y,z = road.st_to_xyz(float(cw.s), float(cw.t), float(cw.zOffset))
                print(f"Converted (s,t,zOffset) to (x,y,z): ({cw.s:.2f}, {cw.t:.2f}, {cw.zOffset:.2f}) --> ({x:.2f}, {y:.2f}, {z:.2f})")

            except Exception as e:
                print(f"Failed: {cw.id_}")
                traceback.print_exc()
            
            print()
            
            for outline in cw.outlines:
                for corner_local in outline:
                    u = corner_local['u']
                    v = corner_local['v']
                    z_local = corner_local['z']

                    #Test uvz
                    try:
                        x,y,z = road.uv_to_xyz(float(cw.s), float(cw.t), float(cw.zOffset), u, v, z_local, float(cw.hdg), float(cw.pitch), float(cw.roll))
                        print(f"Converted (u,v,z_local) to (x,y,z): ({u}, {v}, {z_local}) --> ({x:.2f}, {y:.2f}, {z:.2f})")
                    
                    except Exception as e:
                        print(f"Failed: {cw.id_}")
                        traceback.print_exc()

            print()


def main(path):
    road_map = xodr_parser.RoadMap()
    road_map.parse(path)

    for road_id, road in road_map.roads.items():
        verify_crosswalks(road_id, road)

        

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Invalid Usage: python convert_st_to_xyz_test.py path_to_file.xodr")
    else:
        main(sys.argv[1])
