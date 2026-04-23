import xml.etree.ElementTree as ET
import os
import numpy as np
import carla
from scenic.core.regions import CircularRegion
from scenic.domains.driving.roads import Lane, Intersection, Road
from scenic.core.object_types import Object


class network_cache():
    def __init__(self, 
                 workspace, 
                 scenic_to_metsr_map_lanes,
                 metsr_represented_roads,
                 radius_search_size=30):
          
            self.workspace = workspace
            self.metsr_represented_roads = metsr_represented_roads
            self.scenic_to_metsr_map_lanes = scenic_to_metsr_map_lanes
            self.radius_search_size=radius_search_size

            self.network_lanes = [*self.workspace.network.lanes]
            self.network_roads = [*self.workspace.network.roads]
            self.network_intersections = [*self.workspace.network.intersections]

            self.scenic_to_metsr_map_roads = {}
            self.intersection_road_links = set([])
            self.populate_scenic_to_metsr_roads()

            self.connected_roads_to_intersections = {}
            self.populate_roads_to_intersections()

            self.obj_road_cache = {}
            self.obj_lane_cache = {}

    # Initialilzation

    def populate_scenic_to_metsr_roads(self) -> None:
        """
        Generate scenic -> METSR mappings for roads
        """
        for road,road_map in self.scenic_to_metsr_map_lanes.items():
            scenic_road = road.split("_")[0]
            metsr_road  = road_map.split("_")[0]
            if metsr_road in self.metsr_represented_roads["orig_id"]:
                if scenic_road not in self.scenic_to_metsr_map_roads:
                    self.scenic_to_metsr_map_roads[scenic_road] = []
                    self.scenic_to_metsr_map_roads[scenic_road].append(metsr_road)
                else:
                    if metsr_road not in self.scenic_to_metsr_map_roads[scenic_road]:
                        self.scenic_to_metsr_map_roads[scenic_road].append(metsr_road)
            else:
                self.intersection_road_links.add(metsr_road)
    
    def populate_roads_to_intersections(self) -> None:
        """
        Populate the dictionary 
        """
        for intersection in [*self.workspace.network.intersections]:
            for lane in intersection.incomingLanes:
                if lane.road not in self.connected_roads_to_intersections:
                    self.connected_roads_to_intersections[lane.road] = [intersection]
                else:
                    self.connected_roads_to_intersections[lane.road].append(intersection)
            for lane in intersection.outgoingLanes:
                if lane not in self.connected_roads_to_intersections:
                    self.connected_roads_to_intersections[lane.road] = [intersection]
                else:
                    self.connected_roads_to_intersections[lane.road].append(intersection)


    """Helpers for generating or collecting map data"""

    def _nearest_lane(self,obj : Object, allow_offlane : bool = True, radius_search_size : int = 50, allow_intersection_links : bool = True) -> Lane | None :        
        """
        Docstring for _nearest_lane
        
        Return the nearest lane to the object
         (1) Checks obj lane cache for previous lane 
         (2) Checks connection lanes otherwise
         (3) Queries Scenic
            (4) If lane is none, selects the closest lane in the neighborhood
                around the car defined by the radius parameter

        :param obj: Scenic Vehicle object to find closest lane for
        :type obj:  Scenic Object
        :param allow_offlane: Flag which denotes whether vehicles should be allowed to deviate from the road
        :type allow_offlane: bool
        :param radius_size: Region around objects position to search for lanes if none is found
        :type radis_size: int
        :allow_intersection_links: Flag to allows the selection of non-METSR recognized lanes
        :type allow_intersection_links: bool
        """
        
        radius_size = radius_search_size if radius_search_size else self.radius_search_size
        
        nearest_lane = None
        if obj in self.obj_lane_cache:
            lane = self.obj_lane_cache[obj]
            if lane.containsPoint(obj.position):
                nearest_lane = lane
            else:
                canidate_lanes = lane.adjacentLanes
                if canidate_lanes is not None:
                    for lane in canidate_lanes:
                        if lane.containsPoint(obj.position):
                            nearest_lane = lane
                       
        if nearest_lane is None or not allow_intersection_links:
            if nearest_lane:
                if self.map_scenic_to_metsr_lanes(nearest_lane) not in self.intersection_road_links:
                    self.obj_lane_cache[obj] = nearest_lane
                    return nearest_lane                    
                
            nearest_lane = obj._lane
            if nearest_lane is None:
                if not allow_offlane:
                    assert nearest_lane, f"Object: {obj.name} is has left the roadway"
                neighborhood = CircularRegion(center=[obj.x,obj.y],radius=radius_size) 
                distances = []
                for lane in self.network_lanes:
                    if neighborhood.intersects(lane):
                        distances.append((lane.distanceTo(obj.position), lane))
                assert len(distances) > 0, f"Object has deviated to far from the roadway : i.e {radius_size/2} meters"
               
                if not allow_intersection_links:
                    for _ in range(len(distances)):
                        distance, nearest_lane = min(distances, key=lambda t: t[0])[:] # min distance over all lanes
                        if self.map_scenic_to_metsr_lanes(nearest_lane) not in self.intersection_road_links:
                            break
                        else:
                            distances.remove((distance, nearest_lane))
                else:
                    nearest_lane = min(distances, key=lambda t: t[0])[1] # min distance over all lanes
       
        self.obj_lane_cache[obj] = nearest_lane
        return nearest_lane
    
    def _nearest_road(self, obj: Object, allow_offroad : bool, radius_size: int = 30) -> Road:
        """
            Docstring for _nearest_road

            :param obj: Object to identify road for
            :type obj: Vehicle Object
            :param allow_offroad: Flag whether offroad vehicles are allowed in this simulation
            :type allow_offroad: Bool
            :param radius_size: Radius size in meters of the viable search space for offroad vehicles
            :type radius_size: Integer

            Return the nearest road on the map for a given object
        """
        nearest_road = None
        if obj in self.obj_road_cache:
            road = self.obj_road_cache[obj]
            if road.containsPoint(obj.position):
                nearest_road = road
 
        if not nearest_road and allow_offroad:
            distances = []
            neighborhood = CircularRegion(center=[obj.x,obj.y],radius=radius_size) 
            for road in self.network_roads:
                if neighborhood.intersects(road):
                        if road.containsPoint(obj.position):
                            nearest_road = road
                            break
                        else:
                            distances.append((road.distanceTo(obj.position), road))
            if not nearest_road:
                if len(distances) > 0:
                    nearest_road = min(distances, key=lambda t: t[0])[1] # min distance over all lanes
                else:
                    assert f"Object has deviated to far from the roadway : i.e {radius_size/2} meters"
        
        self.obj_road_cache[obj] = road
        return nearest_road
    
    def _get_intersection(self, obj: Object, road: Road ) -> Intersection | None:
        """
            Docstring for _get_intersection

            :param obj: Object to identify lane for
            :type obj: Vehicle Object
            :param road: Objects current road in map
            :type road: riad

            Collects the nearest lane for an object from the intersection cache based
            on outgoing roads, if none is found queries Scenic
        """
        curr_intersection = None
        if road in self.connected_roads_to_intersections:
            for intersection in self.connected_roads_to_intersections[road]:
                if intersection.containsPoint(obj.position):
                    curr_intersection = intersection
                    break

        return curr_intersection
    
    def _get_bubble_roads(self, bubble_region: CircularRegion) -> list[Road]:
        """
        docstring for get_bubble_roads

        :return: The current set of roads intersecting the CoSimulation bubble
        :rtype: list[road]
        """
        bubble_roads = []
        for road in self.network_roads:
            if road.intersects(bubble_region):
                bubble_roads.append(road)
        return bubble_roads
    

    
    def generate_scenic_trajectory(self, curr_lane, route):
        # Enforce that the first trajectory target corresponds to current location
        metsr_curr_road = self.map_scenic_to_metsr_lanes(curr_lane)
        if metsr_curr_road != route[0]:
            route.insert(0, metsr_curr_road)

        # Collect All valid spawn locations
        map_data = self.scenic_to_metsr_map_lanes.items()
        valid_lanes = {}
        for road in route: 
            for scenic_key, metsr_key in map_data: # Scenic <-> Metsr mappings
                    key_road = metsr_key.split("_")[0] # Road for road_lane pair
                    if key_road == road:
                        if road not in valid_lanes:
                            valid_lanes[road] = []
                        valid_lanes[road].append(scenic_key) 
        
        # Search for corresponding lane with correct direction/orientation (Greedily takes the first lane)
        trajectory = []        
        for i,road in enumerate(route):
            target_lanes = valid_lanes[road]
            assert len(target_lanes) > 1,  f"Failed to find target lanes for road: {road}"
            for road_lane in target_lanes:
                if len(trajectory) == i+1: # Break once lane is collected
                    break
                for lane in self.network_lanes:
                    scenic_road = f'{lane.road.id}'
                    query_road = road_lane.split("_")[0] # Road Key
                    query_lane = road_lane.split("_")[1] # Lane Key
                    opposite_traffic_flag = bool(query_lane[0] == "-")
                    if query_road == scenic_road: # Road Match
                        # Collect the correct lane on road which matches target directoin
                        if opposite_traffic_flag and str(lane.id)[0] == "-":
                            trajectory.append(lane)
                            break
                        elif not opposite_traffic_flag and not str(lane.id)[0] == "-":
                            trajectory.append(lane)
                            break
      
        if len(trajectory) < 1:
            print(f'No roads found')
            return None
                    
        return trajectory
    
    """ Translating Scenic -> Metsr representations"""
    
    def map_scenic_to_metsr_road(self, road):
        query_key = f'{road.id}' 
        metsr_keys= None
        
        if query_key in self.scenic_to_metsr_map_roads: 
            metsr_keys = self.scenic_to_metsr_map_roads[query_key]

        assert metsr_keys is not None, f"Error identifying associated ID for {query_key}"  
        return metsr_keys
    
    def map_scenic_to_metsr_lanes(self, lane):
        metsr_key=None
        query_key = f'{lane.road.id}_{lane.id}' 
        
        # Check if element is present in map between formats
        if query_key in self.scenic_to_metsr_map_lanes: 
            metsr_key = self.scenic_to_metsr_map_lanes[query_key]
        
        metsr_key = metsr_key.split("_")[0]
        # There must be a valid mapping 
        assert metsr_key is not None, f"Error identifying associated ID for {query_key}"    
        return metsr_key

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
    # if verbose:
    #     print(f"checking distance between obj: {object} and cars {[(car.name, car.position) for car in cars]}")
    object_pos = np.array(object.position)
    obj_distances = {}
    for car in cars:
        if car != object:
            threshold = 1.2 * object.length
            dist = np.linalg.norm(np.array(car.position) - object_pos)
            if dist < threshold:
                is_close=True
            obj_distances[car.name] = dist
    if verbose and is_close:
        print(f"Min Distance for {object} was: {min(obj_distances.values())}")
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
    