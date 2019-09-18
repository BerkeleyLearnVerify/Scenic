import scenic.simulators.carla.xodr_parser as xodr_parser
from scenic.core.workspaces import Workspace
from scenic.core.regions import PolygonalRegion
from scenic.core.vectors import VectorField

class CarlaWorkspace(Workspace):
    def __init__(self, path, n=20):
        '''Initialize from OpenDrive file at @path, with
        @n points per lane section reference line.'''
        self.road_map = xodr_parser.RoadMap()
        self.road_map.parse(path)
        self.road_map.calculate_geometry(n)
        drivable_poly = self.road_map.drivable_region
        sidewalk_poly = self.road_map.sidewalk_region
        self.road_direction = VectorField('Road Direction',
                                          self.road_map.heading_at)
        self.drivable_region = PolygonalRegion(polygon=drivable_poly,
                                               orientation=self.road_direction)
        self.sidewalk_region = PolygonalRegion(polygon=sidewalk_poly)
        # lane_sec_dict is dict of road id to list of dict of lane id to PolygonalRegion.
        self.lane_sec_dict = {}
        for id_ in self.road_map.roads:
            lane_dicts = []
            for d in self.road_map.roads[id_].sec_lane_polys:
                lane_dicts.append({i: PolygonalRegion(polygon=d[i],
                                                      orientation=self.road_direction)
                                   for i in d.keys()})
            self.lane_sec_dict[id_] = lane_dicts

    def show(self, plt):
        xodr_parser.plot_poly(self.drivable_region.polygons)
        xodr_parser.plot_poly(self.sidewalk_region.polygons, 'b')

    @property
    def minimumZoomSize(self):
        return 100
