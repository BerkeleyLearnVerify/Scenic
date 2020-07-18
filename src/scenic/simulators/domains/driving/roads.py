
from __future__ import annotations  # allow forward references for type annotations

import enum
import math
from typing import FrozenSet, Union, Tuple, Optional
import itertools
import pathlib
import bz2
import pickle
import pickletools

import attr
from shapely.geometry import Polygon, MultiPolygon

from scenic.core.distributions import distributionFunction
from scenic.core.vectors import Vector, VectorField
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.object_types import Point
import scenic.core.geometry as geometry
import scenic.core.utils as utils

## Typing

# TODO allow additional types which are coercible to vectors?
Vectorlike = Union[Vector, Point]

def toVector(thing: Vectorlike) -> Vector:
    if not hasattr(thing, 'toVector'):
        raise TypeError(f'argument {thing} is not a vector')
    return thing.toVector()

## Metadata

@enum.unique
class VehicleType(enum.Enum):
    CAR = 1
    BICYCLE = 2
    PEDESTRIAN = 3

@enum.unique
class ManeuverType(enum.Enum):
    STRAIGHT = enum.auto()
    LEFT_TURN = enum.auto()
    RIGHT_TURN = enum.auto()
    U_TURN = enum.auto()

    @staticmethod
    def guessTypeFromLanes(start, end, connecting, turnThreshold=math.radians(20)):
        """For formats lacking turn information, guess it from the geometry."""
        if connecting is None:
            return ManeuverType.STRAIGHT
        if end.road is start.road:
            return ManeuverType.U_TURN

        # Identify turns based on relative heading of start and end of connecting lane
        startDir = connecting.centerline[1] - connecting.centerline[0]
        endDir = connecting.centerline[-1] - connecting.centerline[-2]
        turnAngle = startDir.angleWith(endDir)
        if turnAngle >= turnThreshold:
            return ManeuverType.LEFT_TURN
        elif turnAngle <= -turnThreshold:
            return ManeuverType.RIGHT_TURN
        else:
            return ManeuverType.STRAIGHT

@attr.s(auto_attribs=True, kw_only=True, eq=False)
class Maneuver:
    type: ManeuverType = None      # left turn, right turn, straight, etc.
    startLane: Lane
    endLane: Lane

    # the following attributes are None if startLane directly merges into endLane,
    # rather than connecting via a maneuver through an intersection
    connectingLane: Union[Lane, None] = None
    intersection: Union[Intersection, None] = None

    def __attrs_post_init__(self):
        assert self.type is ManeuverType.STRAIGHT or self.connectingLane is not None

        if self.type is None:   # unknown maneuver type; need to guess from geometry
            ty = ManeuverType.guessTypeFromLanes(self.startLane, self.endLane, self.connectingLane)
            object.__setattr__(self, 'type', ty)

    @property
    @utils.cached
    def conflictingManeuvers(self) -> Tuple[Maneuver]:
        guideway = self.connectingLane
        start = self.startLane
        conflicts = []
        for maneuver in self.intersection.maneuvers:
            if (maneuver.startLane is not start
                and maneuver.connectingLane.intersects(guideway)):
                conflicts.append(maneuver)
        return tuple(conflicts)

## Road networks

@attr.s(auto_attribs=True, kw_only=True, repr=False)
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

    name: str = ''      # human-readable name, if any
    uid: str = None     # unique identifier; from underlying format, if possible
    id: Optional[str] = None    # identifier from underlying format, if any

    ## Traffic info

    #: which vehicles (car, bicycle, etc.) can be here
    vehicleTypes: FrozenSet[VehicleType] = frozenset([VehicleType.CAR])
    #: may be inherited from parent
    speedLimit: Union[float, None] = None
    #: 'roundabout', etc.
    tags: FrozenSet[str] = frozenset()

    def __attrs_post_init__(self):
        assert self.uid is not None or self.id is not None
        if self.uid is None:
            self.uid = self.id

        super().__init__(polygon=self.polygon, orientation=self.orientation, name=self.name)

    def __getstate__(self):
        state = self.__dict__.copy()
        # replace links to network elements by placeholders to prevent deep
        # recursions during pickling; as a result of this, only entire `Network`
        # objects can be properly unpickled
        for key, value in state.items():
            if isinstance(value, NetworkElement):
                state[key] = ElementPlaceholder(value.uid)
        return state

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike) -> Tuple[float]:
        """Get nominal traffic direction(s) at a point in this element.

        There must be at least one such direction. If there are multiple, we
        pick one arbitrarily to be the orientation of the element as a Region.
        (So 'Object in element' will align by default to that orientation.)
        """
        return (self.orientation[toVector(point)],)

    def __repr__(self):
        s = f'<{type(self).__name__} at {hex(id(self))}; '
        if self.name:
            s += f'name="{self.name}", '
        if self.id and self.id != self.uid:
            s += f'id="{self.id}", '
        s += f'uid="{self.uid}">'
        return s

class ElementPlaceholder:
    """Placeholder for a link to a pickled `NetworkElement`."""
    def __init__(self, uid):
        self.uid = uid

@attr.s(auto_attribs=True, kw_only=True, repr=False)
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
        # don't check centerline here since it can lie inside a median, for example
        # (TODO reconsider the decision to have polygon only include drivable areas?)
        assert self.containsRegion(self.leftEdge, tolerance=0.5)
        assert self.containsRegion(self.rightEdge, tolerance=0.5)
        if self.orientation is None:
            self.orientation = VectorField(self.name, self.defaultHeadingAt)

    def defaultHeadingAt(self, point):
        """Default orientation for this LinearElement.

        In general, we align along the nearest segment of the centerline.
        For roads, lane groups, etc., we align along the orientation of the
        lane containing the point.
        """
        point = toVector(point)
        start, end = self.centerline.nearestSegmentTo(point)
        return start.angleTo(end)

    @distributionFunction
    def flowFrom(self, point: Vectorlike, distance: float,
                 steps: Union[int, None] = None,
                 stepSize: Union[float, None] = None) -> Vector:
        """Advance a point along this element by a given distance.

        Equivalent to 'follow element.orientation from point for distance', but
        possibly more accurate. The default implementation uses the forward
        Euler approximation with a step size of 5 meters; subclasses may ignore
        the 'steps' and 'stepSize' parameters if they can compute the flow
        exactly.
        """
        if steps is None:
            if stepSize is None:
                stepSize = 5
            steps = min(4, int((distance / stepSize) + 1))
        return self.orientation.followFrom(toVector(point), distance, steps=steps)

class ContainsCenterline:
    """Mixin which asserts that the centerline is contained in the polygon."""
    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        assert self.containsRegion(self.centerline, tolerance=0.5)

@attr.s(auto_attribs=True, kw_only=True, repr=False)
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
        point = toVector(point)
        if self.forwardLanes and self.forwardLanes.containsPoint(point):
            return self.forwardLanes.orientation[point]
        if self.backwardLanes and self.backwardLanes.containsPoint(point):
            return self.backwardLanes.orientation[point]
        return super().defaultHeadingAt(point)

    @distributionFunction
    def sectionAt(self, point: Vectorlike) -> Union[RoadSection, None]:
        """Get the RoadSection passing through a given point."""
        point = toVector(point)
        for section in self.sections:
            if section.containsPoint(point):
                return section
        return None

    @distributionFunction
    def laneSectionAt(self, point: Vectorlike) -> Union[LaneSection, None]:
        """Get the LaneSection passing through a given point."""
        point = toVector(point)
        lane = self.laneAt(point)
        return None if lane is None else lane.sectionAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        point = toVector(point)
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane
        return None

    @distributionFunction
    def laneGroupAt(self, point: Vectorlike) -> Union[LaneGroup, None]:
        """Get the LaneGroup passing through a given point."""
        point = toVector(point)
        if self.forwardLanes and self.forwardLanes.containsPoint(point):
            return self.forwardLanes
        if self.backwardLanes and self.backwardLanes.containsPoint(point):
            return self.backwardLanes
        return None

    @distributionFunction
    def crossingAt(self, point: Vectorlike) -> Union[PedestrianCrossing, None]:
        """Get the PedestrianCrossing passing through a given point."""
        point = toVector(point)
        for crossing in self.crossings:
            if crossing.containsPoint(point):
                return crossing
        return None

    @distributionFunction
    def shiftLanes(self, point: Vectorlike, offset: int) -> Union[Vector, None]:
        """Find the point equivalent to this one but shifted over some # of lanes."""
        raise NotImplementedError   # TODO implement this

    @property
    def is1Way(self):
        return self.forwardLanes is None or self.backwardLanes is None

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class LaneGroup(LinearElement):
    """A group of parallel lanes with the same type and direction."""

    road: Road          # parent road
    lanes: Tuple[Lane]  # partially ordered, with lane 0 being closest to the curb

    # associated elements not actually part of this group
    sidewalk: Union[Sidewalk, None] = None
    bikeLane: Union[Lane, None] = None

    def defaultHeadingAt(self, point):
        point = toVector(point)
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane.orientation[point]
        return super().defaultHeadingAt(point)

@attr.s(auto_attribs=True, kw_only=True, eq=False, repr=False)
class Lane(ContainsCenterline, LinearElement):
    """A lane for cars, bicycles, or other vehicles."""

    group: LaneGroup            # parent lane group
    road: Road                  # grandparent road
    sections: Tuple[LaneSection]    # sections in order from start to end

    adjacentLanes: Tuple[Lane] = ()     # adjacent lanes of same type, if any

    maneuvers: Tuple[Maneuver] = ()     # possible maneuvers upon reaching the end of this lane

    @distributionFunction
    def sectionAt(self, point: Vectorlike) -> Union[LaneSection, None]:
        """Get the LaneSection passing through a given point."""
        point = toVector(point)
        for section in self.sections:
            if section.containsPoint(point):
                return section
        return None

    # TODO remove hack; freeze all these classes
    __hash__ = object.__hash__

@attr.s(auto_attribs=True, kw_only=True, repr=False)
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
        point = toVector(point)
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane.orientation[point]
        return super().defaultHeadingAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike) -> Union[LaneSection, None]:
        """Get the lane section passing through a given point."""
        point = toVector(point)
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane
        return None

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class LaneSection(ContainsCenterline, LinearElement):
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

    @distributionFunction
    def shiftedBy(self, offset: int) -> Union[LaneSection, None]:
        """Find the lane a given number of lanes over from this lane."""
        current = self
        for i in range(abs(offset)):
            if offset > 0:
                current = current.laneToLeft
            else:
                current = current.laneToRight
            if current is None:
                return None
        return current

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Sidewalk(ContainsCenterline, LinearElement):
    road: Road
    crossings: Tuple[PedestrianCrossing]

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class PedestrianCrossing(ContainsCenterline, LinearElement):
    parent: Union[Road, Intersection]
    startSidewalk: Sidewalk
    endSidewalk: Sidewalk

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Intersection(NetworkElement):
    roads: Tuple[Road]     # in some order, preserving adjacency
    incomingLanes: Tuple[Lane]
    outgoingLanes: Tuple[Lane]
    maneuvers: Tuple[Maneuver]  # all possible maneuvers through the intersection

    crossings: Tuple[PedestrianCrossing]    # also ordered to preserve adjacency

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        for maneuver in self.maneuvers:
            assert maneuver.connectingLane, maneuver

    @property
    def is3Way(self):
        return len(self.roads) == 3
    @property
    def is4Way(self):
        return len(self.roads) == 4

    @distributionFunction
    def maneuversAt(self, point: Vectorlike) -> Set[Maneuver]:
        """Get all maneuvers possible at a given point in the intersection."""
        point = toVector(point)
        possible = set()
        for maneuver in self.maneuvers:
            if maneuver.connectingLane.containsPoint(point):
                possible.add(maneuver)
        return possible

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike) -> Tuple[float]:
        point = toVector(point)
        directions = []
        for maneuver in self.maneuvers:
            lane = maneuver.connectingLane
            if lane.containsPoint(point):
                directions.append(lane.orientation[point])
        return tuple(directions)

    ## FOR LATER

    # signals: Tuple[Union[Signal, None]]

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Network:
    """A road network."""
    elements: Dict[str, NetworkElement]     # indexed by unique ID

    # TODO change these to frozensets once everything is hashable?
    roads: Tuple[Road]
    laneGroups: Tuple[LaneGroup]
    lanes: Tuple[Lane]
    intersections: Tuple[Intersection]
    crossings: Tuple[PedestrianCrossing]
    sidewalks: Tuple[Sidewalk]

    roadSections: Tuple[RoadSection] = None
    laneSections: Tuple[LaneSection] = None

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
        for uid, elem in self.elements.items():
            assert elem.uid == uid

        self.roadSections = tuple(sec for road in self.roads for sec in road.sections)
        self.laneSections = tuple(sec for lane in self.lanes for sec in lane.sections)

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
            self.roadDirection = VectorField('roadDirection', self._defaultRoadDirection)

    def _defaultRoadDirection(self, point):
        """Default value for the roadDirection vector field."""
        point = toVector(point)
        road = self.roadAt(point)
        return 0 if road is None else road.orientation[point]

    pickledExt = '.snet'

    @classmethod
    def fromFile(cls, path, useCache=True, writeCache=True, **kwargs):
        path = pathlib.Path(path)
        ext = path.suffix

        handlers = {    # in order of decreasing priority
            cls.pickledExt: cls.fromPickle,     # pickled native representation
            '.xodr': cls.fromOpenDrive,         # OpenDRIVE
        }

        if ext:
            if useCache:
                # use pickled version of network if it exists
                newPath = path.with_suffix(cls.pickledExt)
                if newPath.exists():
                    path = newPath
                    ext = cls.pickledExt
        else:
            # no extension was given; search through possible formats
            for ext in handlers:
                newPath = path.with_suffix(ext)
                if newPath.exists():
                    path = newPath
                    break

        if ext not in handlers:
            raise RuntimeError(f'unknown type of road network file {path}')

        network = handlers[ext](path, **kwargs)
        if writeCache and ext is not cls.pickledExt:
            network.dumpPickle(path.with_suffix(cls.pickledExt))
        return network

    @classmethod
    def fromPickle(cls, path):
        with bz2.open(path, 'rb') as f:
            network = pickle.load(f)
        # Reconnect links between network elements
        for elem in network.elements.values():
            state = elem.__dict__
            for key, value in state.items():
                if isinstance(value, ElementPlaceholder):
                    state[key] = network.elements[value.uid]
        return network

    @classmethod
    def fromOpenDrive(cls, path, ref_points=20, tolerance=0.05, fill_gaps=True):
        import scenic.simulators.formats.opendrive.xodr_parser as xodr_parser
        road_map = xodr_parser.RoadMap(tolerance=tolerance)
        road_map.parse(path)
        road_map.calculate_geometry(ref_points, calc_gap=fill_gaps, calc_intersect=True)
        return road_map.toScenicNetwork()

    def dumpPickle(self, path):
        path = pathlib.Path(path)
        if not path.suffix:
            path = path.with_suffix(self.pickledExt)
        with bz2.open(path, 'wb') as f:
            data = pickle.dumps(self)
            data = pickletools.optimize(data)
            f.write(data)

    @distributionFunction
    def elementAt(self, point: Vectorlike) -> Union[NetworkElement, None]:
        point = toVector(point)
        road = self.roadAt(point)
        if road is not None:
            return road
        return self.intersectionAt(point)

    @distributionFunction
    def roadAt(self, point: Vectorlike) -> Union[Road, None]:
        """Get the road passing through a given point."""
        point = toVector(point)
        for road in self.roads:
            if road.containsPoint(point):
                return road
        return None

    @distributionFunction
    def laneAt(self, point: Vectorlike) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        point = toVector(point)
        for lane in self.lanes:
            if lane.containsPoint(point):
                return lane
        return None

    @distributionFunction
    def laneSectionAt(self, point: Vectorlike) -> Union[LaneSection, None]:
        """Get the LaneSection passing through a given point."""
        point = toVector(point)
        lane = self.laneAt(point)
        return None if lane is None else lane.sectionAt(point)

    @distributionFunction
    def laneGroupAt(self, point: Vectorlike) -> Union[LaneGroup, None]:
        """Get the LaneGroup passing through a given point."""
        point = toVector(point)
        road = self.roadAt(point)
        return None if road is None else road.laneGroupAt(point)

    @distributionFunction
    def crossingAt(self, point: Vectorlike) -> Union[PedestrianCrossing, None]:
        """Get the PedestrianCrossing passing through a given point."""
        point = toVector(point)
        road = self.roadAt(point)
        return None if road is None else road.crossingAt(point)

    @distributionFunction
    def intersectionAt(self, point: Vectorlike) -> Union[Intersection, None]:
        """Get the intersection at a given point."""
        point = toVector(point)
        for intersection in self.intersections:
            if intersection.containsPoint(point):
                return intersection
        return None

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike) -> Tuple[float]:
        """Get nominal traffic direction(s) at a given point, if any."""
        inter = self.intersectionAt(point)
        if inter is not None:
            return inter.nominalDirectionsAt(point)
        road = self.roadAt(point)
        if road is not None:
            return road.nominalDirectionsAt(point)
        return ()

    def show(self, plt):
        """Render a schematic of the road network for debugging."""
        self.drivableRegion.show(plt, style='r')
        for lane in self.lanes:
            lane.show(plt, style='r--')
        self.walkableRegion.show(plt, style='b')
        self.intersectionRegion.show(plt, style='g')

## FOR LATER

# class Signal:
#     """Traffic lights, stop signs, etc."""
