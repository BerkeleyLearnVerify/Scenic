"""Library for representing road network geometry and traffic information.

A road network is represented by an instance of the :obj:`Network` class, which can
be created from a map file using :obj:`Network.fromFile`.

.. note::

    This library is a prototype under active development. We will try not to make
    backwards-incompatible changes, but the API may not be entirely stable. Some
    network information, such as traffic signals, has not yet been made available.
"""

from __future__ import annotations  # allow forward references for type annotations

import io
import enum
import hashlib
import math
import numbers
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

from scenic.core.distributions import distributionFunction, distributionMethod
from scenic.core.vectors import Vector, VectorField
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.object_types import Point
import scenic.core.geometry as geometry
import scenic.core.utils as utils
from scenic.core.errors import InvalidScenarioError
from scenic.core.distributions import RejectionException, distributionFunction
import scenic.core.type_support as type_support
from scenic.syntax.veneer import verbosePrint
import scenic.syntax.veneer as veneer

## Typing and utilities

#: Alias for types which can be interpreted as positions in Scenic.
#:
#: This includes instances of `Point` and `Object`, and pairs of numbers.
Vectorlike = Union[Vector, Point, Tuple[numbers.Real, numbers.Real]]

def _toVector(thing: Vectorlike) -> Vector:
    return type_support.toVector(thing)

def _rejectSample(message):
    if veneer.isActive():
        raise InvalidScenarioError(message)
    else:
        raise RejectionException(message)

def _rejectIfNonexistent(element, name='network element'):
    if element is None:
        _rejectSample(f'requested {name} does not exist')
    return element

class _ElementReferencer:
    """Mixin class to improve pickling of classes that reference network elements.

    :meta private:
    """
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
                state[key] = _ElementPlaceholder(value.uid)
        return state

class _ElementPlaceholder:
    """Placeholder for a link to a pickled `NetworkElement`.

    :meta private:
    """
    def __init__(self, uid):
        self.uid = uid

## Metadata

@enum.unique
class VehicleType(enum.Enum):
    """A type of vehicle, including pedestrians. Used to classify lanes."""
    CAR = 1
    BICYCLE = 2
    PEDESTRIAN = 3

@enum.unique
class ManeuverType(enum.Enum):
    """A type of `Maneuver`, e.g., going straight or turning left."""
    STRAIGHT = enum.auto()      #: Straight, including one lane merging into another.
    LEFT_TURN = enum.auto()     #: Left turn.
    RIGHT_TURN = enum.auto()    #: Right turn.
    U_TURN = enum.auto()        #: U-turn.

    @staticmethod
    def guessTypeFromLanes(start: Lane, end: Lane, connecting: Union[Lane, None],
                           turnThreshold: float = math.radians(20)):
        """For formats lacking turn information, guess it from the geometry.

        Arguments:
            start: starting lane of the maneuver.
            end: ending lane of the maneuver.
            connecting: connecting lane of the maneuver, if any.
            turnThreshold: angle beyond which to consider a maneuver a turn.
        """
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
class Maneuver(_ElementReferencer):
    """Maneuver()

    A maneuver which can be taken upon reaching the end of a lane.
    """
    type: ManeuverType = None   #: type of maneuver (straight, left turn, etc.)
    startLane: Lane             #: starting lane of the maneuver
    endLane: Lane               #: ending lane of the maneuver

    # the following attributes are None if startLane directly merges into endLane,
    # rather than connecting via a maneuver through an intersection

    #: connecting lane from the start to the end lane, if any (`None` for lane mergers)
    connectingLane: Union[Lane, None] = None
    #: intersection where the maneuver takes place, if any (`None` for lane mergers)
    intersection: Union[Intersection, None] = None

    def __attrs_post_init__(self):
        assert self.type is ManeuverType.STRAIGHT or self.connectingLane is not None

        if self.type is None:   # unknown maneuver type; need to guess from geometry
            ty = ManeuverType.guessTypeFromLanes(self.startLane, self.endLane, self.connectingLane)
            object.__setattr__(self, 'type', ty)

    @property
    @utils.cached
    def conflictingManeuvers(self) -> Tuple[Maneuver]:
        """Maneuvers whose connecting lanes intersect this one's."""
        if not self.connectingLane:
            return ()
        guideway = self.connectingLane
        start = self.startLane
        conflicts = []
        for maneuver in self.intersection.maneuvers:
            if (maneuver.startLane is not start
                and maneuver.connectingLane.centerline.intersects(guideway.centerline)):
                conflicts.append(maneuver)
        return tuple(conflicts)

    @property
    @utils.cached
    def reverseManeuvers(self) -> Tuple[Maneuver]:
    	"""Maneuvers whose start and end roads are the reverse of this one's."""
    	start = self.startLane.road
    	end = self.endLane.road
    	reverses = []
    	for maneuver in self.intersection.maneuvers:
    		if (maneuver.startLane.road is end
    			and maneuver.endLane.road is start):
    			reverses.append(maneuver)
    	return tuple(reverses)

## Road networks

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class NetworkElement(_ElementReferencer, PolygonalRegion):
    """NetworkElement()

    Abstract class for part of a road network.

    Includes roads, lane groups, lanes, sidewalks, pedestrian crossings,
    and intersections.

    This is a subclass of `Region`, so you can do things like ``Car in lane``
    or ``Car on road`` if ``lane`` and ``road`` are elements, as well as computing
    distances to an element, etc.
    """

    # from PolygonalRegion
    polygon: Union[Polygon, MultiPolygon]
    orientation: Optional[VectorField] = None

    name: str = ''      #: Human-readable name, if any.
    #: Unique identifier; from underlying format, if possible.
    #: (In OpenDRIVE, for example, ids are not necessarily unique, so we invent our own.)
    uid: str = None
    id: Optional[str] = None    #: Identifier from underlying format, if any.
    network: Network = None     #: Link to parent network.

    ## Traffic info

    #: Which types of vehicles (car, bicycle, etc.) can be here.
    vehicleTypes: FrozenSet[VehicleType] = frozenset([VehicleType.CAR])
    #: Optional speed limit, which may be inherited from parent.
    speedLimit: Union[float, None] = None
    #: Uninterpreted semantic tags, e.g. 'roundabout'.
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
        pick one arbitrarily to be the orientation of the element as a `Region`.
        (So ``Object in element`` will align by default to that orientation.)
        """
        return (self.orientation[_toVector(point)],)

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
    """LinearElement()

    A part of a road network with (mostly) linear 1- or 2-way flow.

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
        return _rejectIfNonexistent(self._successor, 'successor')

    @property
    def predecessor(self):
        return _rejectIfNonexistent(self._predecessor, 'predecessor')

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        # Check that left and right edges lie inside the element.
        # (don't check centerline here since it can lie inside a median, for example)
        # (TODO reconsider the decision to have polygon only include drivable areas?)
        assert self.containsRegion(self.leftEdge, tolerance=0.5)
        assert self.containsRegion(self.rightEdge, tolerance=0.5)
        if self.orientation is None:
            self.orientation = VectorField(self.name, self._defaultHeadingAt)

    def _defaultHeadingAt(self, point):
        """Default orientation for this LinearElement.

        In general, we align along the nearest segment of the centerline.
        For roads, lane groups, etc., we align along the orientation of the
        lane containing the point.

        :meta private:
        """
        point = _toVector(point)
        start, end = self.centerline.nearestSegmentTo(point)
        return start.angleTo(end)

    @distributionFunction
    def flowFrom(self, point: Vectorlike, distance: float,
                 steps: Union[int, None] = None,
                 stepSize: float = 5) -> Vector:
        """Advance a point along this element by a given distance.

        Equivalent to ``follow element.orientation from point for distance``, but
        possibly more accurate. The default implementation uses the forward
        Euler approximation with a step size of 5 meters; subclasses may ignore
        the **steps** and **stepSize** parameters if they can compute the flow
        exactly.

        Arguments:
            point: point to start from.
            distance: distance to travel.
            steps: number of steps to take, or :obj:`None` to compute the
                number of steps based on the distance (default :obj:`None`).
            stepSize: length used to compute how many steps to take, if **steps** is not
                specified (default 5 meters).
        """
        return self.orientation.followFrom(_toVector(point), distance,
                                           steps=steps, stepSize=stepSize)

class _ContainsCenterline:
    """Mixin which asserts that the centerline is contained in the polygon.

    :meta private:
    """
    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        assert self.containsRegion(self.centerline, tolerance=0.5)

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Road(LinearElement):
    """Road()

    A road consisting of one or more lanes.

    Lanes are grouped into 1 or 2 instances of `LaneGroup`:

        * **forwardLanes**: the lanes going the same direction as the road
        * **backwardLanes**: the lanes going the opposite direction

    One of these may be None if there are no lanes in that direction.

    Because of splits and mergers, the Lanes of a `Road` do not necessarily start
    or end at the same point as the `Road`. Such intermediate branching points
    cause the `Road` to be partitioned into multiple road sections, within which
    the configuration of lanes is fixed.
    """
    #: All lanes of this road, in either direction.
    #:
    #: The order of the lanes is arbitrary. To access lanes in order according to their
    #: geometry, use `LaneGroup.lanes`.
    lanes: Tuple[Lane]

    #: Group of lanes aligned with the direction of the road, if any.
    forwardLanes: Union[LaneGroup, None]
    #: Group of lanes going in the opposite direction, if any.
    backwardLanes: Union[LaneGroup, None]   # lanes going the other direction

    #: All LaneGroups of this road, with `forwardLanes` being first if it exists.
    laneGroups: Tuple[LaneGroup] = None

    #: All sections of this road, ordered from start to end.
    sections: Tuple[RoadSection]

    signals: Tuple[Signal]

    #: All crosswalks of this road, ordered from start to end.
    crossings: Tuple[PedestrianCrossing] = ()

    #: All sidewalks of this road, with the one adjacent to `forwardLanes` being first.
    sidewalks: Tuple[Sidewalk] = None
    #: Possibly-empty region consisting of all sidewalks of this road.
    sidewalkRegion: PolygonalRegion = None

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        lgs = []
        sidewalks = []
        if self.forwardLanes:
            lgs.append(self.forwardLanes)
            if self.forwardLanes._sidewalk:
                sidewalks.append(self.forwardLanes._sidewalk)
        if self.backwardLanes:
            lgs.append(self.backwardLanes)
            if self.backwardLanes._sidewalk:
                sidewalks.append(self.backwardLanes._sidewalk)
        self.laneGroups = tuple(lgs)
        self.sidewalks = tuple(sidewalks)
        self.sidewalkRegion = PolygonalRegion.unionAll(sidewalks)

    def _defaultHeadingAt(self, point):
        point = _toVector(point)
        group = self.laneGroupAt(point)
        if group:
            return group.orientation[point]
        return super()._defaultHeadingAt(point)

    @distributionFunction
    def sectionAt(self, point: Vectorlike, reject=False) -> Union[RoadSection, None]:
        """Get the `RoadSection` passing through a given point."""
        return self.network.findPointIn(point, self.sections, reject)

    @distributionFunction
    def laneSectionAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the `LaneSection` passing through a given point."""
        point = _toVector(point)
        lane = self.laneAt(point, reject=reject)
        return None if lane is None else lane.sectionAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[Lane, None]:
        """Get the `Lane` passing through a given point."""
        return self.network.findPointIn(point, self.lanes, reject)

    @distributionFunction
    def laneGroupAt(self, point: Vectorlike, reject=False) -> Union[LaneGroup, None]:
        """Get the `LaneGroup` passing through a given point."""
        return self.network.findPointIn(point, self.laneGroups, reject)

    @distributionFunction
    def crossingAt(self, point: Vectorlike, reject=False) -> Union[PedestrianCrossing, None]:
        """Get the :obj:`.PedestrianCrossing` passing through a given point."""
        return self.network.findPointIn(point, self.crossings, reject)

    @distributionFunction
    def shiftLanes(self, point: Vectorlike, offset: int) -> Union[Vector, None]:
        """Find the point equivalent to this one but shifted over some # of lanes."""
        raise NotImplementedError   # TODO implement this

    @property
    def is1Way(self) -> bool:
        return self.forwardLanes is None or self.backwardLanes is None

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class LaneGroup(LinearElement):
    """LaneGroup()

    A group of parallel lanes with the same type and direction.
    """

    road: Road          #: Parent road.
    lanes: Tuple[Lane]  #: Lanes, partially ordered with lane 0 being closest to the curb.

    #: Region representing the associated curb, which is not necessarily adjacent if
    #: there are parking lanes or some other kind of shoulder.
    curb: PolylineRegion

    # associated elements not actually part of this group
    _sidewalk: Union[Sidewalk, None] = None     #: Adjacent sidewalk, if any.
    _bikeLane: Union[Lane, None] = None
    _shoulder: Union[Shoulder, None] = None     #: Adjacent shoulder, if any.
    #: Opposite lane group of the same road, if any.
    _opposite: Union[LaneGroup, None] = None

    @property
    def sidewalk(self) -> Sidewalk:
        """The adjacent sidewalk; rejects if there is none."""
        return _rejectIfNonexistent(self._sidewalk, 'sidewalk')

    @property
    def bikeLane(self) -> Lane:
        return _rejectIfNonexistent(self._bikeLane, 'bike lane')

    @property
    def shoulder(self) -> Shoulder:
        """The adjacent shoulder; rejects if there is none."""
        return _rejectIfNonexistent(self._shoulder, 'shoulder')

    @property
    def opposite(self) -> LaneGroup:
        """The opposite lane group of the same road; rejects if there is none."""
        return _rejectIfNonexistent(self._opposite, 'opposite lane group')

    def _defaultHeadingAt(self, point):
        point = _toVector(point)
        lane = self.laneAt(point)
        if lane:
            return lane.orientation[point]
        return super()._defaultHeadingAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[Lane, None]:
        """Get the `Lane` passing through a given point."""
        return self.network.findPointIn(point, self.lanes, reject)

@attr.s(auto_attribs=True, kw_only=True, eq=False, repr=False)
class Lane(_ContainsCenterline, LinearElement):
    """Lane()

    A lane for cars, bicycles, or other vehicles.
    """

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
    """RoadSection()

    Part of a road with a fixed number of lanes.

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

    def _defaultHeadingAt(self, point):
        point = _toVector(point)
        lane = self.laneAt(point)
        if lane:
            return lane.orientation[point]
        return super()._defaultHeadingAt(point)

    @distributionFunction
    def laneAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the lane section passing through a given point."""
        return self.network.findPointIn(point, self.lane, reject)

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class LaneSection(_ContainsCenterline, LinearElement):
    """LaneSection()

    Part of a lane in a single `RoadSection`.

    Since the lane configuration in a `RoadSection` is fixed, a `LaneSection` can have
    at most one adjacent lane to left or right. These are accessible using the
    `laneToLeft` and `laneToRight` properties, which for convenience reject the
    simulation if the desired lane does not exist. If rejection is not desired (for
    example if you want to handle the case where there is no lane to the left yourself),
    you can use the `_laneToLeft` and `_laneToRight` properties instead.
    """

    lane: Lane          #: Parent lane.
    group: LaneGroup    #: Grandparent lane group.
    road: Road          #: Great-grandparent road.

    #: ID number as in OpenDRIVE (number of lanes to left of center, with 1 being the
    # first lane left of the centerline and -1 being the first lane to the right).
    openDriveID: int
    #: Whether this lane has the same direction as its parent road.
    isForward: bool = True

    #: Adjacent lanes of the same type, if any.
    adjacentLanes: Tuple[LaneSection] = ()

    #: Adjacent lane of same type to the left, if any.
    _laneToLeft: Union[LaneSection, None] = None
    #: Adjacent lane of same type to the right, if any.
    _laneToRight: Union[LaneSection, None] = None

    #: Faster adjacent lane of same type, if any.
    #: Could be to left or right depending on the country.
    _fasterLane: Union[LaneSection, None] = None
    #: Slower adjacent lane of same type, if any.
    _slowerLane: Union[LaneSection, None] = None

    @property
    def laneToLeft(self) -> LaneSection:
        """The adjacent lane of the same type to the left; rejects if there is none."""
        return _rejectIfNonexistent(self._laneToLeft, 'lane to left')

    @property
    def laneToRight(self) -> LaneSection:
        """The adjacent lane of the same type to the right; rejects if there is none."""
        return _rejectIfNonexistent(self._laneToRight, 'lane to right')

    @property
    def fasterLane(self) -> LaneSection:
        """The faster adjacent lane of the same type; rejects if there is none."""
        return _rejectIfNonexistent(self._fasterLane, 'faster lane')

    @property
    def slowerLane(self) -> LaneSection:
        """The slower adjacent lane of the same type; rejects if there is none."""
        return _rejectIfNonexistent(self._slowerLane, 'slower lane')

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
class Sidewalk(_ContainsCenterline, LinearElement):
    """Sidewalk()

    A sidewalk.
    """
    road: Road
    crossings: Tuple[PedestrianCrossing]

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class PedestrianCrossing(_ContainsCenterline, LinearElement):
    """PedestrianCrossing()

    A pedestrian crossing (crosswalk).
    """
    parent: Union[Road, Intersection]
    startSidewalk: Sidewalk
    endSidewalk: Sidewalk

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Shoulder(_ContainsCenterline, LinearElement):
    """Shoulder()

    A shoulder of a road, including parking lanes by default.
    """
    road: Road

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Intersection(NetworkElement):
    """Intersection()

    An intersection where multiple roads meet.
    """
    roads: Tuple[Road]     # in some order, preserving adjacency
    incomingLanes: Tuple[Lane]
    outgoingLanes: Tuple[Lane]
    maneuvers: Tuple[Maneuver]  # all possible maneuvers through the intersection

    signals: Tuple[Signal]

    crossings: Tuple[PedestrianCrossing]    # also ordered to preserve adjacency

    def __attrs_post_init__(self):
        super().__attrs_post_init__()
        for maneuver in self.maneuvers:
            assert maneuver.connectingLane, maneuver
            assert self.containsRegion(maneuver.connectingLane, tolerance=0.5)

    @property
    def is3Way(self) -> bool:
        """bool: Whether or not this is a 3-way intersection."""
        return len(self.roads) == 3
    @property
    def is4Way(self) -> bool:
        """bool: Whether or not this is a 4-way intersection."""
        return len(self.roads) == 4

    @property
    def isSignalized(self) -> bool:
        """bool: Whether or not this is a signalized intersection."""
        return len(self.signals) > 0

    @distributionFunction
    def maneuversAt(self, point: Vectorlike) -> List[Maneuver]:
        """Get all maneuvers possible at a given point in the intersection."""
        return self.network._findPointInAll(point, self.maneuvers,
                                            key=lambda m: m.connectingLane)

    @distributionFunction
    def nominalDirectionsAt(self, point: Vectorlike) -> List[float]:
        point = _toVector(point)
        maneuvers = self.maneuversAt(point)
        return [m.connectingLane.orientation[point] for m in maneuvers]

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Signal:
    """Traffic lights, stop signs, etc.

    .. warning::

        Signal parsing is a work in progress and the API is likely to change in the future.
    """

    uid: str = None
    #: ID number as in OpenDRIVE (unique ID of the signal within the database)
    openDriveID: int
    #: Country code of the signal
    country: str
    #: Type identifier according to country code.
    type: str

    @property
    def isTrafficLight(self) -> bool:
        """Whether or not this signal is a traffic light."""
        return self.type == "1000001"

@attr.s(auto_attribs=True, kw_only=True, repr=False)
class Network:
    """Network()

    A road network.

    Networks are composed of roads, intersections, sidewalks, etc., which are all
    instances of `NetworkElement`.

    Road networks can be loaded from standard formats using `Network.fromFile`.
    """

    #: All network elements, indexed by unique ID.
    elements: Dict[str, NetworkElement]

    #: All ordinary roads in the network (i.e. those not part of an intersection).
    roads: Tuple[Road]
    #: All roads connecting one exit of an intersection to another.
    connectingRoads: Tuple[Road]
    #: All roads of either type.
    allRoads: Tuple[Road] = None

    #: All lane groups in the network.
    laneGroups: Tuple[LaneGroup]
    #: All lanes in the network.
    lanes: Tuple[Lane]

    #: All intersections in the network.
    intersections: Tuple[Intersection]
    #: All pedestrian crossings in the network.
    crossings: Tuple[PedestrianCrossing]
    #: All sidewalks in the network.
    sidewalks: Tuple[Sidewalk]
    #: All shoulders in the network (by default, includes parking lanes).
    shoulders: Tuple[Shoulder]

    #: All sections of ordinary roads in the network.
    roadSections: Tuple[RoadSection] = None
    #: All sections of lanes in the network.
    laneSections: Tuple[LaneSection] = None

    #: Whether or not cars drive on the left in this network.
    driveOnLeft: bool = False
    #: Distance tolerance for testing inclusion in network elements.
    tolerance: float = 0

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

    #: Traffic flow vector field aggregated over all roads (0 elsewhere).
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
            for road in self.roads:     # only include curbs of ordinary roads
                if road.forwardLanes:
                    edges.append(road.forwardLanes.curb)
                if road.backwardLanes:
                    edges.append(road.backwardLanes.curb)
            self.curbRegion = PolylineRegion.unionAll(edges)

        if self.roadDirection is None:
            # TODO replace with a PolygonalVectorField for better pruning
            self.roadDirection = VectorField('roadDirection', self._defaultRoadDirection)

    def _defaultRoadDirection(self, point):
        """Default value for the `roadDirection` vector field.

        :meta private:
        """
        point = _toVector(point)
        road = self.roadAt(point)
        return 0 if road is None else road.orientation[point]

    #: File extension for cached versions of processed networks.
    pickledExt = '.snet'

    @classmethod
    def _currentFormatVersion(cls):
        """Version number for the road network format.

        Should be incremented whenever attributes of `Network`, `NetworkElement`, etc.,
        attributes of the underlying Regions, or the serialization process itself are
        changed, so that cached networks will be properly regenerated (rather than being
        unpickled in an inconsistent state and causing errors later). Changes to the map
        geometry calculations should be included, even if the format itself is unchanged.

        :meta private:
        """
        return 17

    class DigestMismatchError(Exception):
        """Exception raised when loading a cached map not matching the original file."""
        pass

    @classmethod
    def fromFile(cls, path, useCache:bool = True, writeCache:bool = True, **kwargs):
        """Create a `Network` from a map file.

        This function calls an appropriate parsing routine based on the extension of the
        given file. Supported map formats are:

            * OpenDRIVE (``.xodr``): `Network.fromOpenDrive`

        See the functions listed above for format-specific options to this function.
        If no file extension is given in **path**, this function searches for any file
        with the given name in one of the formats above (in order).

        Args:
            path: A string or other :term:`path-like object` giving a path to a file.
                If no file extension is included, we search for any file type we know how
                to parse.
            useCache: Whether to use a cached version of the map, if one exists
                and matches the given map file (default true; note that if the map file
                changes, the cached version will still not be used).
            writeCache: Whether to save a cached version of the processed map
                after parsing has finished (default true).
            kwargs: Additional keyword arguments specific to particular map formats.

        Raises:
            FileNotFoundError: no readable map was found at the given path.
            ValueError: the given map is of an unknown format.
        """
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
            raise ValueError(f'unknown type of road network file {path}')

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
    def fromOpenDrive(cls, path, ref_points:int = 20, tolerance:float = 0.05,
                      fill_gaps:bool = True, fill_intersections:bool = True,
                      elide_short_roads:bool = False):
        """Create a `Network` from an OpenDRIVE file.

        Args:
            path: Path to the file, as in `Network.fromFile`.
            ref_points: Number of points to discretize continuous reference lines
                into.
            tolerance: Tolerance for merging nearby geometries.
            fill_gaps: Whether to attempt to fill gaps between adjacent lanes.
            fill_intersections: Whether to attempt to fill gaps inside
                intersections.
            elide_short_roads: Whether to attempt to fix geometry artifacts by
                eliding roads with length less than **tolerance**.
        """
        import scenic.formats.opendrive.xodr_parser as xodr_parser
        road_map = xodr_parser.RoadMap(tolerance=tolerance,
                                       fill_intersections=fill_intersections,
                                       elide_short_roads=elide_short_roads)
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
            if version[0] != cls._currentFormatVersion():
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
                try:
                    network = pickle.load(gf)
                except pickle.UnpicklingError:
                    raise    # propagate unpickling errors
                except Exception as e:
                    # convert various other ways unpickling can fail into a more
                    # standard exception
                    raise pickle.UnpicklingError('unpickling failed') from e

        # Reconnect links between network elements
        def reconnect(thing):
            state = thing.__dict__
            for key, value in state.items():
                if isinstance(value, _ElementPlaceholder):
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
        version = struct.pack('<I', self._currentFormatVersion())
        data = pickle.dumps(self)
        with open(path, 'wb') as f:
            f.write(version)    # uncompressed in case we change compression schemes later
            f.write(digest)     # uncompressed for quick lookup
            with gzip.open(f, 'wb') as gf:
                gf.write(data)

    @distributionMethod
    def findPointIn(self, point: Vectorlike,
                    elems: Sequence[NetworkElement],
                    reject: Union[bool, str]) -> Union[NetworkElement, None]:
        """Find the first of the given elements containing the point.

        Elements which *actually* contain the point have priority; if none contain the
        point, then we search again allowing an error of up to **tolerance**. If there
        are still no matches, we return None, unless **reject** is true, in which case we
        reject the current sample.
        """
        point = _toVector(point)
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
            _rejectSample(message)
        return None

    def _findPointInAll(self, point, things, key=lambda e: e):
        point = _toVector(point)
        found = []
        for thing in things:
            if key(thing).containsPoint(point):
                found.append(thing)
        if not found and self.tolerance > 0:
            for thing in things:
                if key(thing).distanceTo(point) <= self.tolerance:
                    found.append(thing)
        return found

    @distributionMethod
    def elementAt(self, point: Vectorlike, reject=False) -> Union[NetworkElement, None]:
        """Get the highest-level `NetworkElement` at a given point, if any.

        If the point lies in an `Intersection`, we return that; otherwise if the point
        lies in a `Road`, we return that; otherwise we return :obj:`None`, or reject the
        simulation if **reject** is true (default false).
        """
        point = _toVector(point)
        intersection = self.intersectionAt(point)
        if intersection is not None:
            return intersection
        return self.roadAt(point, reject=reject)

    @distributionMethod
    def roadAt(self, point: Vectorlike, reject=False) -> Union[Road, None]:
        """Get the `Road` passing through a given point."""
        return self.findPointIn(point, self.allRoads, reject)

    @distributionMethod
    def laneAt(self, point: Vectorlike, reject=False) -> Union[Lane, None]:
        """Get the `Lane` passing through a given point."""
        return self.findPointIn(point, self.lanes, reject)

    @distributionMethod
    def laneSectionAt(self, point: Vectorlike, reject=False) -> Union[LaneSection, None]:
        """Get the `LaneSection` passing through a given point."""
        point = _toVector(point)
        lane = self.laneAt(point, reject=reject)
        return None if lane is None else lane.sectionAt(point)

    @distributionMethod
    def laneGroupAt(self, point: Vectorlike, reject=False) -> Union[LaneGroup, None]:
        """Get the `LaneGroup` passing through a given point."""
        point = _toVector(point)
        road = self.roadAt(point, reject=reject)
        return None if road is None else road.laneGroupAt(point, reject=reject)

    @distributionMethod
    def crossingAt(self, point: Vectorlike,
                   reject=False) -> Union[PedestrianCrossing, None]:
        """Get the `PedestrianCrossing` passing through a given point."""
        point = _toVector(point)
        road = self.roadAt(point, reject=reject)
        return None if road is None else road.crossingAt(point, reject=reject)

    @distributionMethod
    def intersectionAt(self, point: Vectorlike,
                       reject=False) -> Union[Intersection, None]:
        """Get the `Intersection` at a given point."""
        return self.findPointIn(point, self.intersections, reject)

    @distributionMethod
    def nominalDirectionsAt(self, point: Vectorlike, reject=False) -> Tuple[float]:
        """Get the nominal traffic direction(s) at a given point, if any.

        There can be more than one such direction in an intersection, for example: a car
        at a given point could be going straight, turning left, etc.
        """
        inter = self.intersectionAt(point)
        if inter is not None:
            return inter.nominalDirectionsAt(point)
        road = self.roadAt(point, reject=reject)
        if road is not None:
            return road.nominalDirectionsAt(point)
        return ()

    def show(self):
        """Render a schematic of the road network for debugging.

        If you call this function directly, you'll need to subsequently call
        `matplotlib.pyplot.show` to actually display the diagram.
        """
        import matplotlib.pyplot as plt
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
