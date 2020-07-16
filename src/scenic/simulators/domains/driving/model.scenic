"""Scenic world model for scenarios using the generic driving domain."""

from scenic.simulators.domains.driving.workspace import DrivingWorkspace
import scenic.simulators.domains.driving.network as networkModule

from scenic.simulators.utils.colors import Color

network = networkModule.network
workspace = DrivingWorkspace(network)

road = network.drivableRegion
sidewalk = network.sidewalkRegion
intersection = network.intersectionRegion

roadDirection = network.roadDirection

class DrivingObject:
    """Abstract class providing convenience methods for objects in a road network."""

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
