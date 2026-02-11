import xml.etree.ElementTree as ET
import os
import numpy as np

def generate_map(map):
    try:
        tree = ET.parse(map)
    except FileNotFoundError:
        print(f"Could not find map: {map} from {os.getcwd()}")
        return {}
    
    root = tree.getroot()
    mappings = {}
    edges = root.iterfind("edge")

    for edge in edges: 
        lanes = edge.findall('lane')

        for lane in lanes:
            metrs_lane = lane.attrib.get("id")
            params = lane.findall('param')

            for param in params:
                if param.get('key') == "origId":
                    orig_id = param.get('value')
                    orig_id = orig_id.split()
                    if isinstance(orig_id, list):
                        for id in orig_id:
                            mappings[id] = metrs_lane
                    else:
                        mappings[orig_id] = metrs_lane

    if mappings == {}:
        print(f"An occured attempting to process map: {map}")

    return mappings

def test_mapping(map, test_pairs):
    mappings = generate_map(map)
    for key,value in test_pairs.items():
        if key in mappings:
            if value == mappings[key]:
                print(f"key value pair {key,value} mapped correctly")
            else:
                print(f"expected {value} returned {mappings[key]}")
                print(f"Value {mappings[key]} for key {key} was incorrect with actual value {value}")
        else:
            print(f"Failed on test case {key}, {value}")


def within_threshold_to(object, cars) -> bool:
    # print(f"checking distance between obj: {object} and cars {[car.name for car in cars]}")
    object_pos = np.array(object.position)
    obj_distances = []
    for car in cars:
        threshold = 1.2 * object.length
        dist = np.linalg.norm(np.array(car.position) - object_pos)
        if dist < threshold:
            return True
        obj_distances.append(dist)
    print(f"Object distances for obj: {object.name}: {obj_distances}")
    return False




if __name__ == "__main__":

    map = "Town01.net.xml"

    # key value pairs where key == origID and value == lane id
    town01_test_pairs = {"4_1": "4_2", "0_2 11_-2 8_2": "0_1", "8_-3 11_3 0_-3": "-8_0" }

    test_mapping(map, town01_test_pairs)

    map = "Town02.net.xml"

    # Randomly selected test cases from the file to check accuracy
    town02_test_pairs = {"177_-1": ":132_3_0", "276_-3":":242_2_0", "1_-2 16_-2 12_2 3_-2 15_2": "-1_1"}

    test_mapping(map, town02_test_pairs)