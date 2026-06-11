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

            self.network_lane_sections = [*self.workspace.network.laneSections]
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
                if metsr_road in self.metsr_represented_roads["id_list"]:
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
    
    def _nearest_road(self, obj: Object, allow_offroad: bool=True, radius_size: int = 50) -> Road | None:
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
    
    
    def generate_metsr_trajectory(self, scenic_trajectory: list[LaneSection], obj: Object) -> list[str]:
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

        :param road_ids: List of Scenic road IDs which are to be cosimulated
        :type road_ids:  List[str]
        :param bubble_road_ids: List of METSR road IDs which are to be cosimulated
        :type bubble_road_ids: List[str]

        Repair cosimulated region to ensure that METS-R and SCENIC are in synch
        If a road falls outside the respective region but is included in the equivalent METS-R
        map add it to the represented region
        """
        repaired_bubble = []
        for id in bubble_road_ids:
            all_connected_roads = self.all_scenic_roads_connected_too[id]
            if len(all_connected_roads) > 1:
                for road_id in all_connected_roads:
                    if road_id not in road_ids:
                        repaired_bubble.append(self.roads_by_id[road_id])
        return repaired_bubble

