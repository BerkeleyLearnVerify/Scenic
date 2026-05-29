from scenic.core.regions import CircularRegion
from scenic.domains.driving.roads import LaneSection, Intersection, Road, Lane
from scenic.core.object_types import Object
import matplotlib.pyplot as plt

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

            self.network_lanes = [*self.workspace.network.laneSections]
            self.network_roads = [*self.workspace.network.allRoads]
            self.roads_by_id = {str(road.id): road for road in self.network_roads}
            self.network_intersections = [*self.workspace.network.intersections]

            self.scenic_to_metsr_map_roads = {}
            self.all_scenic_roads_connected_too = {}
            self.intersection_road_links = set([])
            self.scenic_unique_roads = set([])
            self.populate_scenic_to_metsr_roads()

            self.connected_roads_to_intersections = {}
            self.populate_roads_to_intersections()

            self.obj_road_cache = {}
            self.obj_lane_cache = {}

    # Initialilzation
    def _visualize_network(self,bubble_roads: list[Road], ego_position, frozen_metsr_road_lines, count):
        self.workspace.network.show(bubble_roads=bubble_roads, ego_position=ego_position, metsr_road_lines=frozen_metsr_road_lines)
        plt.savefig(f"./figures/town06/run2/network_at_{count}")

    def populate_scenic_to_metsr_roads(self) -> None:
        """
        Generate scenic -> METSR mappings for roads
        """
        all_scenic_roads_connected_too = {}
        for road_lane,road_lane_map in self.scenic_to_metsr_map_lanes.items():
            scenic_road = road_lane.split("_")[0]
            for metsr_map in road_lane_map:
                metsr_road  = metsr_map.split("_")[0]
                if metsr_road in self.metsr_represented_roads["orig_id"]:
                    if scenic_road not in self.scenic_to_metsr_map_roads:
                        self.scenic_to_metsr_map_roads[scenic_road] = set()
                        self.scenic_to_metsr_map_roads[scenic_road].add(metsr_road)
                        if metsr_road not in all_scenic_roads_connected_too:
                            all_scenic_roads_connected_too[metsr_road] = [scenic_road]
                        else:
                            all_scenic_roads_connected_too[metsr_road].append(scenic_road)
                    else:
                        if metsr_road not in self.scenic_to_metsr_map_roads[scenic_road]:
                            self.scenic_to_metsr_map_roads[scenic_road].add(metsr_road)
                            if metsr_road not in all_scenic_roads_connected_too:
                                all_scenic_roads_connected_too[metsr_road] = [scenic_road]
                            else:
                                all_scenic_roads_connected_too[metsr_road].append(scenic_road)
                else:
                    self.intersection_road_links.add(metsr_road)
        
        for road_lane in self.scenic_to_metsr_map_lanes.keys():
            scenic_road = road_lane.split("_")[0]
            if scenic_road not in self.scenic_to_metsr_map_roads:
                self.scenic_unique_roads.add(scenic_road)
        self.all_scenic_roads_connected_too = all_scenic_roads_connected_too
    
    def populate_roads_to_intersections(self) -> None:
        """
        Populate the dictionary 
        """
        for intersection in [*self.workspace.network.intersections]:
            for road in intersection.roads:
                if road not in self.connected_roads_to_intersections:
                    self.connected_roads_to_intersections[road] = [intersection]
                else:
                    self.connected_roads_to_intersections[road].append(intersection)

    """Helpers for generating or collecting map data"""

    def _nearest_lane(self,obj : Object, allow_offlane : bool = True, radius_search_size : int = 50, allow_intersection_links : bool = True) -> LaneSection | None :        
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
                metsr_roads = self.map_scenic_to_metsr_lanes(nearest_lane)
                if metsr_roads:
                    for road in metsr_roads:
                        if road not in self.intersection_road_links:
                            self.obj_lane_cache[obj] = nearest_lane
                            return nearest_lane                    
                
            nearest_lane = obj._laneSection
            if nearest_lane is not None: # TODO need to cleanup this case or make a second helper
                mapped_roads = self.map_scenic_to_metsr_lanes(nearest_lane)
                if mapped_roads: # Check if the returned lane has a valid metsr mapping
                    continue_search = not self.map_scenic_to_metsr_lanes(nearest_lane).isdisjoint(self.intersection_road_links) and not(allow_intersection_links)
                else:        
                    continue_search = True

            if nearest_lane is None or continue_search:
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
                        mapped_roads = self.map_scenic_to_metsr_lanes(nearest_lane)
                        if mapped_roads:
                            if mapped_roads.isdisjoint(self.intersection_road_links):
                                break
                            else:
                                distances.remove((distance, nearest_lane))
                else:
                    nearest_lane = min(distances, key=lambda t: t[0])[1] # min distance over all lanes
       
        self.obj_lane_cache[obj] = nearest_lane

        return nearest_lane
    
    def _nearest_road(self, obj: Object, allow_offroad: bool=True, radius_size: int = 50) -> Road:
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
            if road.containsPoint(obj.position): # try to verify road through the cache
                return road

        nearest_road = obj._road  # last resort lookup
        if nearest_road is not None: # Maintain the previous road
           self.obj_road_cache[obj] = nearest_road
     
        return nearest_road
    
    def _get_intersection(self, obj: Object, curr_road: Road = None ) -> Intersection | None:
        """
            Docstring for _get_intersection

            :param obj: Object to identify lane for
            :type obj: Vehicle Object
            :param road: Objects current road in map
            :type road: road

            Checks if the obj is on an intersection based, on its previous logged road
            Looks up the intersection directly if no log exists yet
        """
        intersection = None
        road = None
        if curr_road: # If obj is on road it is not in an intersection 
            return None
       
        else:          
            pos = obj.position 
            if obj in self.obj_road_cache:
                road = self.obj_road_cache[obj] # find the most recent road
          
        if road in self.connected_roads_to_intersections: # check if obj is in a connected intersection
            intersections = self.connected_roads_to_intersections[road]
            for intersection in intersections:
                if intersection.containsPoint(pos):
                    intersection = intersection
                    break

        if road is None or intersection is None: # Previous road was not cached, so we need to do a global lookup
            intersection = obj._intersection
    
        return intersection
    
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
    
    
    def generate_scenic_trajectory(self, curr_lane: LaneSection, route: list[str] ) -> None | list[LaneSection]:
        """
        docstring for generate_scenic_trajectory

        :param curr_lane: Current lane the object which that targeted route is being generated for is on 
        :type curr_lane:  Lane
        :param route: Proposed route for object queried from METSR with METSR road IDs
        :type route: List[str]

        Attempts to generate an eqivalent route of Scenic Lanes from a list of METSR target road IDs.
        Enforces that the first road in the trajectory corresponds to the objects current road. If
        no route is found returns None. 
        
        """
        # # Enforce that the first trajectory target corresponds to current location
        # metsr_curr_roads = self.map_scenic_to_metsr_lanes(curr_lane) 
        # if metsr_curr_roads:
        #     if route[0] not in metsr_curr_roads:
        #         route.insert(0, metsr_curr_roads.pop()) # Arbritrarily any potential map

        # Collect All valid spawn locations
        map_data = self.scenic_to_metsr_map_lanes.items()
        # print(f"METSR_MAPS: {list(self.scenic_to_metsr_map_lanes.values())}")
        valid_lanes = {}
        for road in route: 
            for scenic_key, metsr_keys in map_data: # Scenic <-> Metsr mappings
                    for metsr_key in metsr_keys:
                        key_road = metsr_key.split("_")[0] # Road for road_lane pair
                        if key_road == road:
                            if road not in valid_lanes:
                                valid_lanes[road] = []
                            valid_lanes[road].append(scenic_key) 
            
        # Search for corresponding lane with correct direction/orientation (Greedily takes the first lane)
        trajectory = [] 
        # print(f'Route: {route}')       
        for i,road in enumerate(route):
            # print(f'Processing road: {road}')
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
        
        final_trajectory = list(set([ls.lane for ls in trajectory])) # Convert to the correct type TODO fix this... 
        return final_trajectory
    
    def generate_metsr_trajectory(self, scenic_trajectory: list[LaneSection]) -> list[str]:
        """
        docstring for generate_metsr_trajectory

        :param trajectory: ordered sequence of target lanesections
        :type trajectory: list[LandSection] 
        """
        metsr_trajectory = []
        for laneSection in scenic_trajectory:
            if laneSection not in self.intersection_road_links:
                mapped_roads = self.scenic_to_metsr_map_lanes(laneSection)
                if mapped_roads:
                    if len(mapped_roads) > 1:
                        print(f'What do you even do in this case: {mapped_roads} ')
                    road = mapped_roads.pop()
                    metsr_trajectory.append(road)
                else:
                    print(f"Skipped non existing mapping for laneSection: {laneSection.road.id}_{laneSection.lane.id}")
        return metsr_trajectory
    

    """ Translating Scenic -> Metsr representations"""
    
    def map_scenic_to_metsr_road(self, road : Road) -> list[str] | None:
        """
        docstring for map_scenic_to_metsr_road
        
        :param road: Scenic road ID which should be translated to equivalent METSR road ID(s)
        :type road: Road

        """
        if str(road.id) in self.scenic_unique_roads:
            return None
     
        query_key = f'{road.id}' 
        metsr_keys= None
        
        if query_key in self.scenic_to_metsr_map_roads: 
            metsr_keys = self.scenic_to_metsr_map_roads[query_key]

        assert metsr_keys is not None, f"Error identifying associated ID for {query_key}"  
        return metsr_keys
    
    def map_scenic_to_metsr_lanes(self, lane: LaneSection) -> set[str] | None:
        """
        docstring for map_scenic_to_metsr_lanes

        Given a OpenDrive LaneSection
        
        """
        metsr_keys=None
        query_key = f'{lane.road.id}_{lane.id}' 
        
        if query_key in self.scenic_to_metsr_map_lanes: 
            metsr_keys = self.scenic_to_metsr_map_lanes[query_key]
            metsr_keys = set([metsr_key.split("_")[0] for metsr_key in metsr_keys])

        return metsr_keys
    
    def repair_mapping(self,road_ids, bubble_road_ids):
        """
        Docstring for repair_mappping

        :param road_ids:
        :type road_ids:
        :param bubble_road_ids:
        :type bubble_road_ids:
        
        """
        repaired_bubble = []
        for id in bubble_road_ids:
            all_connected_roads = self.all_scenic_roads_connected_too[id]
            if len(all_connected_roads) > 1:
                for road_id in all_connected_roads:
                    if road_id not in road_ids:
                        repaired_bubble.append(self.roads_by_id[road_id])
        return repaired_bubble

