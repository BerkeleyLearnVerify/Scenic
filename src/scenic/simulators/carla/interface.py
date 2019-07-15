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
        self.road_direction = VectorField('Road Direction',
                                          self.road_map.heading_at)
        self.drivable_region = PolygonalRegion(polygon=drivable_poly,
                                               orientation=self.road_direction)

    def show(self, plt):
        xodr_parser.plot_poly(self.drivable_region.polygons)

    @property
    def minimumZoomSize(self):
        return 100
