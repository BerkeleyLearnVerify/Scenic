
from __future__ import annotations  # allow forward references for type annotations

import enum
from typing import FrozenSet, Union, Tuple, Optional
import itertools
import pathlib

import attr
from shapely.geometry import Polygon, MultiPolygon

from scenic.core.vectors import Vector, VectorField
from scenic.core.regions import PolygonalRegion, PolylineRegion

## Metadata

@enum.unique
class VehicleType(enum.Enum):
    CAR = 1
    BICYCLE = 2
    PEDESTRIAN = 3

## Road networks

@attr.s(auto_attribs=True, kw_only=True)
class NetworkElement(PolygonalRegion):
    """Abstract class for part of a road network.

    Includes roads, lane groups, lanes, sidewalks, pedestrian crossings,
    and intersections.

    This is a subclass of Region, so you can do things like 'Car in lane'
    or 'Car on road' if 'lane' and 'road' are elements, as well as computing
    distances to an element, etc.
    """

    # from PolygonalRegion
    polygon: Union[Polygon, MultiPolygon]
    orientation: Optional[VectorField] = None

    name: str = ''
    id: Optional[str] = None    # unique identifier from underlying format

    ## Traffic info

    #: which vehicles (car, bicycle, etc.) can be here
    vehicleTypes: FrozenSet[VehicleType] = frozenset([VehicleType.CAR])
    #: may be inherited from parent
    speedLimit: Union[float, None] = None
    #: 'roundabout', etc.
    tags: FrozenSet[str] = frozenset()

    def __attrs_post_init__(self):
        super().__init__(polygon=self.polygon, orientation=self.orientation)

@attr.s(auto_attribs=True, kw_only=True)
class LinearElement(NetworkElement):
    """A part of a road network with (mostly) linear 1- or 2-way flow.

    Includes roads, lane groups, lanes, sidewalks, and pedestrian crossings,
    but not intersections.

    LinearElements have a direction, namely from the first point on their centerline
    to the last point. This is called 'forward', even for 2-way roads. The 'left' and
    'right' edges are interpreted with respect to this direction.
    """

    # Geometric info (on top of the overall polygon from PolygonalRegion)
    centerline: PolylineRegion
    leftEdge: PolylineRegion
    rightEdge: PolylineRegion

    # Links to next/previous element
    successor: Union[NetworkElement, None] = None   # going forward
    predecessor: Union[NetworkElement, None] = None # going backward

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        if self.orientation is None:
            self.orientation = VectorField(self.name, self.defaultHeadingAt)

    def defaultHeadingAt(self, point):
        """Default orientation for this LinearElement.

        In general, we align along the nearest segment of the centerline.
        For roads, lane groups, etc., we align along the orientation of the
        lane containing the point.
        """
        pts = [Vector(pt[0], pt[1]) for pt in self.centerline.points]
        distances = [point.distanceTo(pt) for pt in pts]
        i, closest = min(enumerate(pts), key=lambda i_pt: distances[i_pt[0]])
        if i == 0:
            j = 1
        elif i == len(self.centerline.points)-1:
            j = i
            i -= 1
        elif distances[i+1] >= distances[i-1]:
            j = i+1
        else:
            j = i
            i -= 1
        return pts[i].angleTo(pts[j])

    def nominalDirectionsAt(point: Vector) -> Tuple[float]:
        """Get nominal traffic direction(s) at a point in this element.

        There must be at least one such direction. If there are multiple, we
        pick one arbitrarily to be the orientation of the element as a Region.
        (So 'Object in element' will align by default to that orientation.)
        """

    def flowFrom(point: Vector, distance: float) -> Vector:
        """Advance a point along this element by a given distance.

        Equivalent to 'follow element.orientation from point for distance'.
        """

@attr.s(auto_attribs=True, kw_only=True)
class Road(LinearElement):
    """A road consisting of one or more lanes.

    Lanes are grouped into 1 or 2 LaneGroups:
        * forwardLanes: the lanes going the same direction as the road
        * backwardLanes: the lanes going the opposite direction
    One of these may be None if there are no lanes in that direction.

    Because of splits and mergers, the Lanes of a Road do not necessarily start
    or end at the same point as the Road. Such intermediate branching points
    cause the Road to be partitioned into multiple RoadSections, within which
    the configuration of lanes is fixed.
    """
    lanes: Tuple[Lane]
    forwardLanes: Union[LaneGroup, None]    # lanes aligned with the direction of the road
    backwardLanes: Union[LaneGroup, None]   # lanes going the other direction
    sections: Tuple[RoadSection]    # sections in order from start to end

    crossings: Tuple[PedestrianCrossing] = ()    # ordered from start to end

    def defaultHeadingAt(self, point):
        if self.forwardLanes and self.forwardLanes.containsPoint(point):
            return self.forwardLanes.orientation[point]
        if self.backwardLanes and self.backwardLanes.containsPoint(point):
            return self.backwardLanes.orientation[point]
        return super().defaultHeadingAt(point)

    def sectionAt(point: Vector) -> Union[RoadSection, None]:
        """Get the RoadSection passing through a given point."""
        for section in self.sections:
            if section.containsPoint(point):
                return section
        return None

    def laneAt(point: Vector) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane
        return None

    def shiftLanes(point: Vector, offset: int) -> Union[Vector, None]:
        """Find the point equivalent to this one but shifted over some # of lanes."""

    @property
    def is1Way(self):
        return self.forwardLanes is None or self.backwardLanes is None

@attr.s(auto_attribs=True, kw_only=True)
class LaneGroup(LinearElement):
    """A group of parallel lanes with the same type and direction."""

    road: Road          # parent road
    lanes: Tuple[Lane]  # partially ordered, with lane 0 being closest to the curb

    # associated elements not actually part of this group
    sidewalk: Union[Sidewalk, None] = None
    bikeLane: Union[Lane, None] = None

    def defaultHeadingAt(self, point):
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane.orientation[point]
        return super().defaultHeadingAt(point)

@attr.s(auto_attribs=True, kw_only=True)
class Lane(LinearElement):
    """A lane for cars, bicycles, or other vehicles."""

    group: LaneGroup            # parent lane group
    road: Road                  # grandparent road
    sections: Tuple[LaneSection]    # sections in order from start to end

    adjacentLanes: Tuple[Lane] = ()    # adjacent lanes of same type, if any

    # for later
    #maneuvers: FrozenSet[Maneuver]      # possible maneuvers upon reaching the end of this lane

@attr.s(auto_attribs=True, kw_only=True)
class RoadSection(LinearElement):
    """Part of a road with a fixed number of lanes.

    A RoadSection has a fixed number of lanes: when a lane begins or ends, we
    move to a new section (which will be the successor of the current one).
    """

    road: Road      # parent road
    lanes: Tuple[LaneSection] = ()   # in order, with lane 0 being the rightmost
    forwardLanes: Tuple[LaneSection] = ()   # as above
    backwardLanes: Tuple[LaneSection] = ()  # as above

    lanesByOpenDriveID: Dict[LaneSection]

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        if not self.lanes and not self.lanesByOpenDriveID:
            raise RuntimeError('RoadSection created with no lanes')
        if self.lanesByOpenDriveID and not self.lanes:
            forward, backward = [], []
            rightmost = min(self.lanesByOpenDriveID)
            assert rightmost != 0, self.lanesByOpenDriveID
            leftmost = max(self.lanesByOpenDriveID)
            for i in range(rightmost, leftmost+1):
                if i == 0:
                    continue
                if i not in self.lanesByOpenDriveID:
                    continue
                (forward if i < 0 else backward).append(self.lanesByOpenDriveID[i])
            self.forwardLanes = tuple(forward)
            self.backwardLanes = tuple(backward)
            self.lanes = self.forwardLanes + self.backwardLanes
        elif self.lanes and not self.lanesByOpenDriveID:
            ids = {}
            for i, lane in enumerate(self.forwardLanes, start=1):
                ids[-i] = lane
            for i, lane in enumerate(self.backwardLanes, start=1):
                ids[i] = lane
            self.lanesByOpenDriveID = ids

    def defaultHeadingAt(self, point):
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane.orientation[point]
        return super().defaultHeadingAt(point)

@attr.s(auto_attribs=True, kw_only=True)
class LaneSection(LinearElement):
    """Part of a lane in a single RoadSection."""

    lane: Lane          # parent lane
    group: LaneGroup    # grandparent group
    road: Road          # great-grandparent road

    openDriveID: int        # ID number as in OpenDRIVE (positive left of center, negative right)
    isForward: bool = True  # whether aligned with parent road

    adjacentLanes: Tuple[LaneSection] = ()     # adjacent lanes of same type, if any

    laneToLeft: Union[LaneSection, None] = None   # adjacent lane of same type to the left, if any
    laneToRight: Union[LaneSection, None] = None

    fasterLane: Union[LaneSection, None] = None   # faster/slower adjacent lane, if it exists;
    slowerLane: Union[LaneSection, None] = None   # could be to left or right depending on country

    def shiftedBy(offset: int) -> Union[LaneSection, None]:
        """Find the lane a given number of lanes over from this lane."""

@attr.s(auto_attribs=True, kw_only=True)
class Sidewalk(LinearElement):
    road: Road
    crossings: Tuple[PedestrianCrossing]

@attr.s(auto_attribs=True, kw_only=True)
class PedestrianCrossing(LinearElement):
    parent: Union[Road, Intersection]
    startSidewalk: Sidewalk
    endSidewalk: Sidewalk

@attr.s(auto_attribs=True, kw_only=True)
class Intersection(NetworkElement):
    roads: Tuple[Road]     # in some order, preserving adjacency
    incomingLanes: Tuple[Lane]
    outgoingLanes: Tuple[Lane]

    crossings: Tuple[PedestrianCrossing]    # also ordered to preserve adjacency

    # include utility properties like this?
    @property
    def is3Way(self):
        return len(self.roads) == 3
    @property
    def is4Way(self):
        return len(self.roads) == 4

    ## FOR LATER

    # signals: Tuple[Union[Signal, None]]

    # def maneuversAt(point: Vector): -> Set[Maneuver]
    #     """Get all maneuvers possible at a given point in the intersection."""

@attr.s(auto_attribs=True, kw_only=True)
class Network:
    """A road network."""
    # TODO change these to frozensets once everything is hashable?
    roads: Tuple[Road]
    laneGroups: Tuple[LaneGroup]
    lanes: Tuple[Lane]
    intersections: Tuple[Intersection]
    crossings: Tuple[PedestrianCrossing]
    sidewalks: Tuple[Sidewalk]

    driveOnLeft: bool = False

    # convenience regions aggregated from various types of network elements
    drivableRegion: PolygonalRegion = None
    walkableRegion: PolygonalRegion = None
    roadRegion: PolygonalRegion = None
    laneRegion: PolygonalRegion = None
    intersectionRegion: PolygonalRegion = None
    crossingRegion: PolygonalRegion = None
    sidewalkRegion: PolygonalRegion = None

    # traffic flow vector field aggregated over all roads (0 elsewhere)
    roadDirection: VectorField = None

    def __attrs_post_init__(self):
        if self.roadRegion is None:
            self.roadRegion = PolygonalRegion.unionAll(self.roads)
        if self.laneRegion is None:
            self.laneRegion = PolygonalRegion.unionAll(self.lanes)
        if self.intersectionRegion is None:
            self.intersectionRegion = PolygonalRegion.unionAll(self.intersections)
        if self.crossingRegion is None:
            self.crossingRegion = PolygonalRegion.unionAll(self.crossings)
        if self.sidewalkRegion is None:
            self.sidewalkRegion = PolygonalRegion.unionAll(self.sidewalks)

        if self.drivableRegion is None:
            self.drivableRegion = self.laneRegion.union(self.intersectionRegion)
        if self.walkableRegion is None:
            self.walkableRegion = self.sidewalkRegion.union(self.crossingRegion)

        if self.roadDirection is None:
            def headingAt(point):
                for road in self.roads:
                    if road.containsPoint(point):
                        return road.orientation[point]
                return 0
            self.roadDirection = VectorField('roadDirection', headingAt)

    @classmethod
    def fromFile(cls, path, **kwargs):
        path = pathlib.Path(path)
        ext = path.suffix
        if ext == '.xodr':
            return cls.fromOpenDrive(path, **kwargs)
        else:
            raise RuntimeError(f'unknown type of road network file {path}')

    @classmethod
    def fromOpenDrive(cls, path, ref_points=20, tolerance=0.05, fill_gaps=True):
        import scenic.simulators.formats.opendrive.xodr_parser as xodr_parser
        road_map = xodr_parser.RoadMap(tolerance=tolerance)
        road_map.parse(path)
        road_map.calculate_geometry(ref_points, calc_gap=fill_gaps, calc_intersect=True)
        return road_map.toScenicNetwork()

    def elementAt(point: Vector) -> Union[NetworkElement, None]:
        road = self.roadAt(point)
        if road is not None:
            return road
        return self.intersectionAt(point)

    def roadAt(point: Vector) -> Union[Road, None]:
        """Get the road passing through a given point."""
        for road in self.roads:
            if road.containsPoint(point):
                return road
        return None

    def laneAt(point: Vector) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane
        return None

    def intersectionAt(point: Vector) -> Union[Intersection, None]:
        """Get the intersection at a given point."""
        for intersection in self.intersections:
            if intersection.containsPoint(point):
                return intersection
        return None

## FOR LATER

# class Maneuver:
#     type: ManeuverType      # left turn, right turn, straight
#     startLane: Lane
#     endLane: Lane
#     guideway: Guideway      # use class from Intelligent Intersections Toolkit?

# class Signal:
#     """Traffic lights, stop signs, etc."""
