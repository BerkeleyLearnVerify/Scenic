"""Scenic world model for scenarios using the generic driving domain."""

from scenic.simulators.domains.driving.workspace import DrivingWorkspace
import scenic.simulators.domains.driving.network as networkModule

from scenic.simulators.utils.colors import Color

network = networkModule.network
workspace = DrivingWorkspace(network)

road = network.drivableRegion
curb = network.curbRegion
sidewalk = network.sidewalkRegion
intersection = network.intersectionRegion

roadDirection = network.roadDirection

class DrivingObject:
    """Abstract class for objects in a road network.

    Provides convenience properties for the lane, road, intersection, etc. at the
    object's current position (if any).

    Also defines the 'elevation' property as a standard way to access the Z
    component of an object's position, since the Scenic built-in property
    'position' is only 2D. If 'elevation' is set to `None`, the simulator is
    responsible for choosing an appropriate Z coordinate so that the object is
    on the ground, then updating the property. 2D simulators should set the
    property to zero.
    """

    elevation[dynamic]: None

    @property
    def lane(self):
        return network.laneAt(self)

    @property
    def laneSection(self):
        return network.laneSectionAt(self)

    @property
    def laneGroup(self):
        return network.laneGroupAt(self)

    @property
    def road(self):
        return network.roadAt(self)

    @property
    def intersection(self):
        return network.intersectionAt(self)

    @property
    def crossing(self):
        return network.crossingAt(self)

    @property
    def element(self):
        return network.elementAt(self)

class Vehicle(DrivingObject):
    regionContainedIn: road
    position: Point on road
    heading: (roadDirection at self.position) + self.roadDeviation
    roadDeviation: 0
    viewAngle: 90 deg
    color: [1, 0, 0]

class Car(Vehicle):
    width: 2
    height: 4.5
    color: Color.defaultCarColor()

class Pedestrian(DrivingObject):
    regionContainedIn: network.walkableRegion
    position: Point on network.walkableRegion
    heading: (0, 360) deg
    viewAngle: 90 deg
    width: 0.75
    height: 0.75
    color: [0, 0.5, 1]
