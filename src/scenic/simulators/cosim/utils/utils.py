import xml.etree.ElementTree as ET
import os
import numpy as np
import carla


def generate_map(map):
    try:
        tree = ET.parse(map)
    except FileNotFoundError:
        print(f"Could not find map: {map} from {os.getcwd()}")
        return {}
    
    root = tree.getroot()
    lane_mappings = {}
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
                            lane_mappings[id] = metrs_lane
                    else:
                        lane_mappings[orig_id] = metrs_lane

    if lane_mappings == {}:
        print(f"An occured attempting to process map: {map}")

    return lane_mappings

def generate_signal_map(map):
   
    try:
        tree = ET.parse(map)
    except FileNotFoundError:
        print(f"Could not find map: {map} from {os.getcwd()}")
        return {}
   
    root = tree.getroot()
    signal_mappings = {}
    
    for tl in root.findall(".//tlLogic"):
        tl_id = tl.attrib.get("id")
        for param in tl.findall('param'): 
            
            key = param.attrib.get("key","")

            if key.startswith("linkSignalID:"):
                metsr_key = f"{tl_id}_{key.split(':')[1]}"
            
                values = param.attrib.get("value","").split()
                if values != "":
                    signal_mappings[metsr_key] = values
    
    return signal_mappings


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


def within_threshold_to(object, cars, verbose=False) -> bool:
    is_close = False
    if verbose:
        print(f"checking distance between obj: {object} and cars {[(car.name, car.position) for car in cars]}")
    object_pos = np.array(object.position)
    obj_distances = []
    for car in cars:
        if car != object:
            threshold = 1.2 * object.length
            dist = np.linalg.norm(np.array(car.position) - object_pos)
            if dist < threshold:
                is_close=True
            obj_distances.append(dist)
    if verbose:
        print(f"Distances were: {obj_distances}")
    return is_close

def get_metsr_rotation(carla_yaw):
    """
    Invert carla_yaw = (bearing - 90) % 360
    to recover the original METSR compass bearing.
    """
    # ensure 0 ≤ yaw < 360
    carla_yaw = carla_yaw % 360
    # invert the shift of -90°
    return (carla_yaw + 90) % 360

def get_carla_light_state(light) -> dict:
    light_state_dict = {'green_time':light.get_green_time(),
                        'red_time':   light.get_red_time(),
                        'yellow_time':light.get_yellow_time(),
                        'state'      :light.get_state() }
    
    return light_state_dict

def disable_carla_autopilot(self, obj) -> bool:
    if hasattr(obj, 'carlaActor'):
        if obj.carlaActor != None:
            obj.carlaActor.set_autopilot(False)
            return True
    else:
        return False
    
def _snapToGround(world, location, blueprint):
    """Mutates @location to have the same z-coordinate as the nearest waypoint in @world."""
    waypoint = world.get_map().get_waypoint(location)
    # patch to avoid the spawn error issue with vehicles and walkers.
    z_offset = 0
    if blueprint is not None and ("vehicle" in blueprint or "walker" in blueprint):
        z_offset = 0.5

    location.z = waypoint.transform.location.z + z_offset
    return location
    
def scenicToCarlaLocation(pos, world=None, blueprint=None, snapToGround=False):
    if snapToGround:
        assert world is not None
        return _snapToGround(world, carla.Location(pos.x, -pos.y, 0.0), blueprint)
    return carla.Location(pos.x, -pos.y, pos.z)

def scenicToCarlaRotation(orientation):
    # CARLA uses intrinsic yaw, pitch, roll rotations (in that order), like Scenic,
    # but with yaw being left-handed and with zero yaw being East.
    yaw, pitch, roll = orientation.r.as_euler("ZXY", degrees=True)
    yaw = -yaw - 90
    return carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)

    




if __name__ == "__main__":

    map = "Town01.net.xml"

    # key value pairs where key == origID and value == lane id
    town01_test_pairs = {"4_1": "4_2", "0_2 11_-2 8_2": "0_1", "8_-3 11_3 0_-3": "-8_0" }

    test_mapping(map, town01_test_pairs)

    map = "Town02.net.xml"

    # Randomly selected test cases from the file to check accuracy
    town02_test_pairs = {"177_-1": ":132_3_0", "276_-3":":242_2_0", "1_-2 16_-2 12_2 3_-2 15_2": "-1_1"}

    test_mapping(map, town02_test_pairs)

    map = "Town05.net.xml"

    result = generate_signal_map(map)

    print(f'Result was: {result}')
    