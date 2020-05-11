"""Workspaces based on OpenDRIVE maps."""

from .xodr_parser import RoadMap
from scenic.core.workspaces import Workspace
from scenic.core.regions import regionFromShapelyObject, nowhere
from scenic.core.vectors import VectorField

class OpenDriveWorkspace(Workspace):
    def __init__(self, path, n=20, tolerance=None):
        '''Initialize from OpenDRIVE file at @path, with
        @n points per lane section reference line.'''
        self.road_map = RoadMap(tolerance=tolerance)
        self.road_map.parse(path)
        self.road_map.calculate_geometry(n, calc_intersect=True)
        drivable_poly = self.road_map.drivable_region
        sidewalk_poly = self.road_map.sidewalk_region
        intersect_poly = self.road_map.intersection_region
        self.road_direction = VectorField('Road Direction',
                                          self.road_map.heading_at)
        self.drivable_region = regionFromShapelyObject(drivable_poly,
                                                       orientation=self.road_direction)
        self.sidewalk_region = regionFromShapelyObject(sidewalk_poly)
        self.intersection_region = regionFromShapelyObject(intersect_poly,
                                                           orientation=self.road_direction)
        super().__init__()

        # lane_sec_dict is dict of road id to list of dict of lane id to Region.
        self.lane_sec_dict = {}
        for id_ in self.road_map.roads:
            lane_dicts = []
            for d in self.road_map.roads[id_].sec_lane_polys:
                lane_dicts.append({i: regionFromShapelyObject(d[i],
                                                              orientation=self.road_direction)
                                   for i in d.keys()})
            self.lane_sec_dict[id_] = lane_dicts

    def show(self, plt):
        self.drivable_region.show(plt)
        self.sidewalk_region.show(plt, style='b')
        self.intersection_region.show(plt, style='g')
