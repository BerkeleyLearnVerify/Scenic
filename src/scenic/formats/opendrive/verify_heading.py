from scenic.formats.opendrive import xodr_parser

import math
import sys

store_differences = []

def test_heading(curve, label, epsilon=2):
    length = curve.length
    if length <= 0:
        print("Bad Curve")
    
    s = length/2 #Just using the midpoint
    s0 = max(0, s - epsilon)
    s1 = min(curve.length, s + epsilon)

    if s1 <= s0:
        print(f"Epsilon too short: {epsilon}")
    
    x0, y0, _ = curve.point_at(s0)
    x1, y1, _ = curve.point_at(s1)

    dx = x1 - x0
    dy = y1 - y0

    aprox_heading = math.atan2(dy, dx) #What we just calculated using the two very close points to s 
    heading = curve.heading_at(s) #Formal function for calculating heading for each curve
    difference = abs(aprox_heading - heading)
    store_differences.append(difference)

    print("Curve Length = ", length)
    print("s = ", s)
    print(f"Curve Attributes: {curve.__dict__}")
    print(f"Label: {label}, Aprox Heading: {aprox_heading:.6f}, Heading: {heading:.6f}, Difference: {difference:.6f}")
    print()

def main(path, sample_size=1000):
    road_map = xodr_parser.RoadMap()
    road_map.parse(path)
    road_count = 0 

    for road_id, road in road_map.roads.items():
        if road.ref_line: #List of Curve objects so we don't have to manually define a Curve
            for curve in road.ref_line:
                label = f"Road {road_id}, Curve: {curve.__class__.__name__}"
                if curve.__class__.__name__ == "Line" or (curve.__class__.__name__ == "Clothoid" and abs(curve.curv1 - curve.curv0) <= 1e-9):
                    continue
                test_heading(curve, label)
            
            road_count = road_count + 1
            if road_count > sample_size:
                break
    if store_differences:
        print("Smallest Difference: ", min(store_differences))
        print("Largest Difference: ", max(store_differences))

if __name__ == "__main__":
    main(sys.argv[1])