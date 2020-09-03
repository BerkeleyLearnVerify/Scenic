
from __future__ import annotations  # allow forward references for type annotations

import io
import enum
import hashlib
import math
from typing import FrozenSet, Union, Tuple, Optional, Sequence, List
import itertools
import pathlib
import gzip
import pickle
import time
import struct
import weakref

import attr
from shapely.geometry import Polygon, MultiPolygon

from scenic.core.distributions import distributionFunction
from scenic.core.vectors import Vector, VectorField
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.object_types import Point
import scenic.core.geometry as geometry
import scenic.core.utils as utils
from scenic.core.errors import InvalidScenarioError
from scenic.core.distributions import RejectionException, distributionFunction
from scenic.syntax.veneer import verbosePrint
import scenic.syntax.veneer as veneer

## Typing and utilities

# TODO allow additional types which are coercible to vectors?
Vectorlike = Union[Vector, Point]

def toVector(thing: Vectorlike) -> Vector:
    if not hasattr(thing, 'toVector'):
        raise TypeError(f'argument {thing} is not a vector')
    return thing.toVector()

def rejectSample(message):
    if veneer.isActive():
        raise InvalidScenarioError(message)
    else:
        raise RejectionException(message)

def rejectIfNonexistent(element, name='network element'):
    if element is None:
        rejectSample(f'requested {name} does not exist')
    return element

class ElementReferencer:
    """Mixin class to improve pickling of classes that reference network elements."""
    def __getstate__(self):
        if hasattr(super(), '__getstate__'):
            state = super().__getstate__()
        else:
            state = self.__dict__.copy()
        # replace links to network elements by placeholders to prevent deep
        # recursions during pickling; as a result of this, only entire `Network`
        # objects can be properly unpickled
        for key, value in state.items():
            if isinstance(value, NetworkElement):
                state[key] = ElementPlaceholder(value.uid)
        return state

class ElementPlaceholder:
    """Placeholder for a link to a pickled `NetworkElement`."""
    def __init__(self, uid):
        self.uid = uid

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
class Maneuver(ElementReferencer):
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
class NetworkElement(ElementReferencer, PolygonalRegion):
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
    network: Network = None     # link to parent network

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

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike) -> Tuple[float]:
        """Get nominal traffic direction(s) at a point in this element.

        There must be at least one such direction. If there are multiple, we
        pick one arbitrarily to be the orientation of the element as a Region.
        (So 'Object in element' will align by default to that orientation.)
        """
        return (self.orientation[toVector(point)],)

    def __getstate__(self):
        state = super().__getstate__()
        del state['network']    # do not pickle weak reference to parent network
        return state

    def __repr__(self):
        s = f'<{type(self).__name__} at {hex(id(self))}; '
        if self.name:
            s += f'name="{self.name}", '
        if self.id and self.id != self.uid:
            s += f'id="{self.id}", '
        s += f'uid="{self.uid}">'
        return s

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class LinearElement(NetworkElement):
    """A part of a road network with (mostly) linear 1- or 2-way flow.

    Includes roads, lane groups, lanes, sidewalks, and pedestrian crossings,
    but not intersections.

    LinearElements have a direction, namely from the first point on their centerline
    to the last point. This is called 'forward', even for 2-way roads. The 'left' and
    'right' edges are interpreted with respect to this direction.

    The left/right edges are oriented along the direction of traffic near them; so
    for 2-way roads they will point opposite directions.
    """

    # Geometric info (on top of the overall polygon from PolygonalRegion)
    centerline: PolylineRegion
    leftEdge: PolylineRegion
    rightEdge: PolylineRegion

    # Links to next/previous element
    _successor: Union[NetworkElement, None] = None   # going forward
    _predecessor: Union[NetworkElement, None] = None # going backward

    @property
    def successor(self):
        return rejectIfNonexistent(self._successor, 'successor')

    @property
    def predecessor(self):
        return rejectIfNonexistent(self._predecessor, 'predecessor')

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        # Check that left and right edges lie inside the element.
        # (don't check centerline here since it can lie inside a median, for example)
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
                 stepSize: Union[float, None] = 5) -> Vector:
        """Advance a point along this element by a given distance.

        Equivalent to 'follow element.orientation from point for distance', but
        possibly more accurate. The default implementation uses the forward
        Euler approximation with a step size of 5 meters; subclasses may ignore
        the 'steps' and 'stepSize' parameters if they can compute the flow
        exactly.
        """
        return self.orientation.followFrom(toVector(point), distance,
                                           steps=steps, stepSize=None)

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
    laneGroups: Tuple[LaneGroup] = None
    sections: Tuple[RoadSection]    # sections in order from start to end

    crossings: Tuple[PedestrianCrossing] = ()    # ordered from start to end

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        lgs = []
        if self.forwardLanes:
            lgs.append(self.forwardLanes)
        if self.backwardLanes:
            lgs.append(self.backwardLanes)
        self.laneGroups = tuple(lgs)

    def defaultHeadingAt(self, point):
        point = toVector(point)
        group = self.laneGroupAt(point)
        if group:
            return group.orientation[point]
        return super().defaultHeadingAt(point)

    @distributionFunction
    def sectionAt(self, point: Vectorlike, reject=False) -> Union[RoadSection, None]:
        """Get the RoadSection passing through a given point."""
        return self.network.findPointIn(point, self.sections, reject)

    @distributionFunction
    def laneSectionAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the LaneSection passing through a given point."""
        point = toVector(point)
        lane = self.laneAt(point, reject=reject)
        return None if lane is None else lane.sectionAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        return self.network.findPointIn(point, self.lanes, reject)

    @distributionFunction
    def laneGroupAt(self, point: Vectorlike, reject=False) -> Union[LaneGroup, None]:
        """Get the LaneGroup passing through a given point."""
        return self.network.findPointIn(point, self.laneGroups, reject)

    @distributionFunction
    def crossingAt(self, point: Vectorlike, reject=False) -> Union[PedestrianCrossing, None]:
        """Get the PedestrianCrossing passing through a given point."""
        return self.network.findPointIn(point, self.crossings, reject)

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
    curb: PolylineRegion

    # associated elements not actually part of this group
    _sidewalk: Union[Sidewalk, None] = None
    _bikeLane: Union[Lane, None] = None
    _shoulder: Union[Shoulder, None] = None
    _opposite: Union[LaneGroup, None] = None

    @property
    def sidewalk(self):
        return rejectIfNonexistent(self._sidewalk, 'sidewalk')

    @property
    def bikeLane(self):
        return rejectIfNonexistent(self._bikeLane, 'bike lane')

    @property
    def shoulder(self):
        return rejectIfNonexistent(self._shoulder, 'shoulder')

    @property
    def opposite(self):
        return rejectIfNonexistent(self._opposite, 'opposite lane group')

    def defaultHeadingAt(self, point):
        point = toVector(point)
        lane = self.laneAt(point)
        if lane:
            return lane.orientation[point]
        return super().defaultHeadingAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        return self.network.findPointIn(point, self.lanes, reject)

@attr.s(auto_attribs=True, kw_only=True, eq=False, repr=False)
class Lane(ContainsCenterline, LinearElement):
    """A lane for cars, bicycles, or other vehicles."""

    group: LaneGroup            # parent lane group
    road: Road                  # grandparent road
    sections: Tuple[LaneSection]    # sections in order from start to end

    adjacentLanes: Tuple[Lane] = ()     # adjacent lanes of same type, if any

    maneuvers: Tuple[Maneuver] = ()     # possible maneuvers upon reaching the end of this lane

    @distributionFunction
    def sectionAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the LaneSection passing through a given point."""
        return self.network.findPointIn(point, self.sections, reject)

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
        lane = self.laneAt(point)
        if lane:
            return lane.orientation[point]
        return super().defaultHeadingAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the lane section passing through a given point."""
        return self.network.findPointIn(point, self.lane, reject)

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class LaneSection(ContainsCenterline, LinearElement):
    """Part of a lane in a single RoadSection."""

    lane: Lane          # parent lane
    group: LaneGroup    # grandparent group
    road: Road          # great-grandparent road

    openDriveID: int        # ID number as in OpenDRIVE (positive left of center, negative right)
    isForward: bool = True  # whether aligned with parent road

    adjacentLanes: Tuple[LaneSection] = ()     # adjacent lanes of same type, if any

    _laneToLeft: Union[LaneSection, None] = None   # adjacent lane of same type to the left, if any
    _laneToRight: Union[LaneSection, None] = None

    _fasterLane: Union[LaneSection, None] = None   # faster/slower adjacent lane, if it exists;
    _slowerLane: Union[LaneSection, None] = None   # could be to left or right depending on country

    @property
    def laneToLeft(self):
        return rejectIfNonexistent(self._laneToLeft, 'lane to left')

    @property
    def laneToRight(self):
        return rejectIfNonexistent(self._laneToRight, 'lane to right')

    @property
    def fasterLane(self):
        return rejectIfNonexistent(self._fasterLane, 'faster lane')

    @property
    def slowerLane(self):
        return rejectIfNonexistent(self._slowerLane, 'slower lane')

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
class Shoulder(ContainsCenterline, LinearElement):
    road: Road

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
            assert self.containsRegion(maneuver.connectingLane, tolerance=0.5)

    @property
    def is3Way(self):
        return len(self.roads) == 3
    @property
    def is4Way(self):
        return len(self.roads) == 4

    @distributionFunction
    def maneuversAt(self, point: Vectorlike) -> List[Maneuver]:
        """Get all maneuvers possible at a given point in the intersection."""
        return self.network.findPointInAll(point, self.maneuvers,
                                           key=lambda m: m.connectingLane)

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike) -> List[float]:
        point = toVector(point)
        maneuvers = self.maneuversAt(point)
        return [m.connectingLane.orientation[point] for m in maneuvers]

    ## FOR LATER

    # signals: Tuple[Union[Signal, None]]

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Network:
    """A road network."""

    elements: Dict[str, NetworkElement]     # indexed by unique ID

    # TODO change these to frozensets once everything is hashable?
    roads: Tuple[Road]              # ordinary roads
    connectingRoads: Tuple[Road]    # roads inside intersections
    allRoads: Tuple[Road] = None    # both kinds of roads

    laneGroups: Tuple[LaneGroup]
    lanes: Tuple[Lane]

    intersections: Tuple[Intersection]
    crossings: Tuple[PedestrianCrossing]
    sidewalks: Tuple[Sidewalk]
    shoulders: Tuple[Shoulder]

    roadSections: Tuple[RoadSection] = None
    laneSections: Tuple[LaneSection] = None

    driveOnLeft: bool = False
    tolerance: float = 0        # tolerance for testing inclusion in elements

    # convenience regions aggregated from various types of network elements
    drivableRegion: PolygonalRegion = None
    walkableRegion: PolygonalRegion = None
    roadRegion: PolygonalRegion = None
    laneRegion: PolygonalRegion = None
    intersectionRegion: PolygonalRegion = None
    crossingRegion: PolygonalRegion = None
    sidewalkRegion: PolygonalRegion = None
    curbRegion: PolylineRegion = None
    shoulderRegion: PolygonalRegion = None

    # traffic flow vector field aggregated over all roads (0 elsewhere)
    roadDirection: VectorField = None

    def __attrs_post_init__(self):
        proxy = weakref.proxy(self)
        for uid, elem in self.elements.items():
            assert elem.uid == uid
            elem.network = proxy

        self.allRoads = self.roads + self.connectingRoads
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
        if self.shoulderRegion is None:
            self.shoulderRegion = PolygonalRegion.unionAll(self.shoulders)

        if self.drivableRegion is None:
            self.drivableRegion = self.laneRegion.union(self.intersectionRegion)
        if self.walkableRegion is None:
            self.walkableRegion = self.sidewalkRegion.union(self.crossingRegion)

        if self.curbRegion is None:
            edges = []
            for group in self.laneGroups:
                edges.append(group.curb)
            self.curbRegion = PolylineRegion.unionAll(edges)

        if self.roadDirection is None:
            self.roadDirection = VectorField('roadDirection', self._defaultRoadDirection)

    def _defaultRoadDirection(self, point):
        """Default value for the roadDirection vector field."""
        point = toVector(point)
        road = self.roadAt(point)
        return 0 if road is None else road.orientation[point]

    pickledExt = '.snet'

    @classmethod
    def currentFormatVersion(cls):
        """Version number for the road network format.

        Should be incremented whenever attributes of `Network`, `NetworkElement`, etc.,
        attributes of the underlying Regions, or the serialization process itself are
        changed, so that cached networks will be properly regenerated (rather than being
        unpickled in an inconsistent state and causing errors later). Changes to the map
        geometry calculations should be included, even if the format itself is unchanged.
        """
        return 12

    class DigestMismatchError(Exception):
        pass

    @classmethod
    def fromFile(cls, path, useCache=True, writeCache=True, **kwargs):
        path = pathlib.Path(path)
        ext = path.suffix

        handlers = {    # in order of decreasing priority
            '.xodr': cls.fromOpenDrive,         # OpenDRIVE

            # Pickled native representation; this is the lowest priority, since original
            # maps should take precedence, but if the pickled version exists and matches
            # the original, we'll use it.
            cls.pickledExt: cls.fromPickle
        }

        if not ext:     # no extension was given; search through possible formats
            found = False
            for ext in handlers:
                newPath = path.with_suffix(ext)
                if newPath.exists():
                    path = newPath
                    found = True
                    break
            if not found:
                raise FileNotFoundError(f'no readable maps found for path {path}')
        elif ext not in handlers:
            raise RuntimeError(f'unknown type of road network file {path}')

        # If we don't have an underlying map file, return the pickled version directly
        if ext == cls.pickledExt:
            return cls.fromPickle(path)

        # Otherwise, hash the underlying file to detect when the pickle is outdated
        with open(path, 'rb') as f:
            data = f.read()
        digest = hashlib.blake2b(data).digest()

        # By default, use the pickled version if it exists and is not outdated
        pickledPath = path.with_suffix(cls.pickledExt)
        if useCache and pickledPath.exists():
            try:
                return cls.fromPickle(pickledPath, originalDigest=digest)
            except pickle.UnpicklingError:
                verbosePrint('Unable to load cached network (old format or corrupted).')
            except cls.DigestMismatchError:
                verbosePrint('Cached network does not match original file; ignoring it.')

        # Not using the pickled version; parse the original file based on its extension
        network = handlers[ext](path, **kwargs)
        if writeCache:
            verbosePrint(f'Caching road network in {cls.pickledExt} file.')
            network.dumpPickle(path.with_suffix(cls.pickledExt), digest)
        return network

    @classmethod
    def fromOpenDrive(cls, path, ref_points=20, tolerance=0.05,
                      fill_gaps=True, fill_intersections=True):
        import scenic.formats.opendrive.xodr_parser as xodr_parser
        road_map = xodr_parser.RoadMap(tolerance=tolerance,
                                       fill_intersections=fill_intersections)
        startTime = time.time()
        verbosePrint('Parsing OpenDRIVE file...')
        road_map.parse(path)
        verbosePrint('Computing road geometry... (this may take a while)')
        road_map.calculate_geometry(ref_points, calc_gap=fill_gaps, calc_intersect=True)
        network = road_map.toScenicNetwork()
        totalTime = time.time() - startTime
        verbosePrint(f'Finished loading OpenDRIVE map in {totalTime:.2f} seconds.')
        return network

    @classmethod
    def fromPickle(cls, path, originalDigest=None):
        startTime = time.time()
        verbosePrint('Loading cached version of road network...')

        with open(path, 'rb') as f:
            versionField = f.read(4)
            if len(versionField) != 4:
                raise pickle.UnpicklingError(f'{cls.pickledExt} file is corrupted')
            version = struct.unpack('<I', versionField)
            if version[0] != cls.currentFormatVersion():
                raise pickle.UnpicklingError(f'{cls.pickledExt} file is too old; '
                                             'regenerate it from the original map')
            digest = f.read(64)
            if len(digest) != 64:
                raise pickle.UnpicklingError(f'{cls.pickledExt} file is corrupted')
            if originalDigest and originalDigest != digest:
                raise cls.DigestMismatchError(
                    f'{cls.pickledExt} file does not correspond to the original map; '
                    ' regenerate it'
                )
            with gzip.open(f) as gf:
                network = pickle.load(gf)

        # Reconnect links between network elements
        def reconnect(thing):
            state = thing.__dict__
            for key, value in state.items():
                if isinstance(value, ElementPlaceholder):
                    state[key] = network.elements[value.uid]
        proxy = weakref.proxy(network)
        for elem in network.elements.values():
            reconnect(elem)
            elem.network = proxy
        for elem in itertools.chain(network.lanes, network.intersections):
            for maneuver in elem.maneuvers:
                reconnect(maneuver)

        totalTime = time.time() - startTime
        verbosePrint(f'Loaded cached network in {totalTime:.2f} seconds.')
        return network

    def dumpPickle(self, path, digest):
        path = pathlib.Path(path)
        if not path.suffix:
            path = path.with_suffix(self.pickledExt)
        version = struct.pack('<I', self.currentFormatVersion())
        data = pickle.dumps(self)
        with open(path, 'wb') as f:
            f.write(version)    # uncompressed in case we change compression schemes later
            f.write(digest)     # uncompressed for quick lookup
            with gzip.open(f, 'wb') as gf:
                gf.write(data)

    def findPointIn(self, point: Vectorlike,
                    elems: Sequence[NetworkElement],
                    reject: Union[bool, str]) -> Union[NetworkElement, None]:
        """Find the first of the given elements containing the point.

        Elements which *actually* contain the point have priority; if none contain the
        point, then we search again allowing an error of up to `tolerance`. If there are
        still no matches, we return None, unless ``reject`` is true, in which case we
        reject the current sample.
        """
        point = toVector(point)
        for element in elems:
            if element.containsPoint(point):
                return element
        if self.tolerance > 0:
            for element in elems:
                if element.distanceTo(point) <= self.tolerance:
                    return element
        if reject:
            if isinstance(reject, str):
                message = reject
            else:
                message = 'requested element does not exist'
            rejectSample(message)
        return None

    def findPointInAll(self, point, things, key=lambda e: e):
        point = toVector(point)
        found = []
        for thing in things:
            if key(thing).containsPoint(point):
                found.append(thing)
        if not found and self.tolerance > 0:
            for thing in things:
                if key(thing).distanceTo(point) <= self.tolerance:
                    found.append(thing)
        return found

    @distributionFunction
    def elementAt(self, point: Vectorlike, reject=False) -> Union[NetworkElement, None]:
        point = toVector(point)
        intersection = self.intersectionAt(point)
        if intersection is not None:
            return intersection
        return self.roadAt(point, reject=reject)

    @distributionFunction
    def roadAt(self, point: Vectorlike, reject=False) -> Union[Road, None]:
        """Get the road passing through a given point."""
        return self.findPointIn(point, self.allRoads, reject)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[Lane, None]:
        """Get the lane passing through a given point."""
        return self.findPointIn(point, self.lanes, reject)

    @distributionFunction
    def laneSectionAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the LaneSection passing through a given point."""
        point = toVector(point)
        lane = self.laneAt(point, reject=reject)
        return None if lane is None else lane.sectionAt(point)

    @distributionFunction
    def laneGroupAt(self, point: Vectorlike, reject=False) -> Union[LaneGroup, None]:
        """Get the LaneGroup passing through a given point."""
        point = toVector(point)
        road = self.roadAt(point, reject=reject)
        return None if road is None else road.laneGroupAt(point, reject=reject)

    @distributionFunction
    def crossingAt(self, point: Vectorlike,
                   reject=False) -> Union[PedestrianCrossing, None]:
        """Get the PedestrianCrossing passing through a given point."""
        point = toVector(point)
        road = self.roadAt(point, reject=reject)
        return None if road is None else road.crossingAt(point, reject=reject)

    @distributionFunction
    def intersectionAt(self, point: Vectorlike,
                       reject=False) -> Union[Intersection, None]:
        """Get the intersection at a given point."""
        return self.findPointIn(point, self.intersections, reject)

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike, reject=False) -> Tuple[float]:
        """Get nominal traffic direction(s) at a given point, if any."""
        inter = self.intersectionAt(point)
        if inter is not None:
            return inter.nominalDirectionsAt(point)
        road = self.roadAt(point, reject=reject)
        if road is not None:
            return road.nominalDirectionsAt(point)
        return ()

    def show(self, plt):
        """Render a schematic of the road network for debugging."""
        self.walkableRegion.show(plt, style='-', color='#00A0FF')
        self.shoulderRegion.show(plt, style='-', color='#606060')
        for road in self.roads:
            road.show(plt, style='r-')
            for lane in road.lanes:     # will loop only over lanes of main roads
                lane.leftEdge.show(plt, style='r--')
                lane.rightEdge.show(plt, style='r--')

                # Draw arrows indicating road direction
                if lane.centerline.length >= 40:
                    pts = lane.centerline.equallySpacedPoints(20)
                else:
                    pts = [lane.centerline.pointAlongBy(0.5, normalized=True)]
                hs = [lane.centerline.orientation[pt] for pt in pts]
                x, y = zip(*pts)
                u = [math.cos(h + (math.pi/2)) for h in hs]
                v = [math.sin(h + (math.pi/2)) for h in hs]
                plt.quiver(x, y, u, v,
                           pivot='middle', headlength=4.5,
                           scale=0.06, units='dots', color='#A0A0A0')
        for lane in self.lanes:     # draw centerlines of all lanes (including connecting)
            lane.centerline.show(plt, style=':', color='#A0A0A0')
        self.intersectionRegion.show(plt, style='g')
        # for intersection in self.intersections:
        #     for i, lane in enumerate(intersection.incomingLanes):
        #         x, y = lane.centerline[-1]
        #         plt.plot([x], [y], '*b')
        #         plt.annotate(str(i), (x, y))

## FOR LATER

# class Signal:
#     """Traffic lights, stop signs, etc."""
