from pathlib import Path

import pytest

from scenic.core.distributions import RejectionException
from scenic.domains.driving.roads import Intersection, Network
from tests.domains.driving.conftest import mapFolder

# Suppress all warnings from OpenDRIVE parser
pytestmark = pytest.mark.filterwarnings(
    "ignore::scenic.formats.opendrive.OpenDriveWarning"
)


def test_network_invalid():
    with pytest.raises(ValueError):
        Network.fromFile("blogobloggo_zoggle.foobar")  # unknown file extension
    with pytest.raises(FileNotFoundError):
        Network.fromFile("blogobloggo_zoggle")  # search all known formats


@pytest.mark.graphical
def test_show2D(network):
    import matplotlib.pyplot as plt

    network.show(labelIncomingLanes=True)
    plt.show(block=False)
    plt.close()


def test_element_tolerance(cached_maps, pytestconfig):
    path = cached_maps[str(mapFolder / "CARLA" / "Town01.xodr")]
    tol = 0.05
    network = Network.fromFile(path, tolerance=tol)
    drivable = network.drivableRegion
    toofar = drivable.buffer(2 * tol).difference(drivable.buffer(1.5 * tol))
    toofar_noint = toofar.difference(network.intersectionRegion)
    road = network.roads[0]
    nearby = road.buffer(tol).difference(road)
    rounds = 30 if pytestconfig.getoption("--fast") else 300
    for i in range(rounds):
        pt = None
        while not pt or pt in drivable:
            pt = nearby.uniformPointInner()
        assert network.elementAt(pt) is not None
        assert network.roadAt(pt) is not None
        pt = toofar.uniformPointInner()
        assert network.roadAt(pt) is None
        with pytest.raises(RejectionException):
            network.roadAt(pt, reject=True)
        pt = toofar_noint.uniformPointInner()
        assert network.elementAt(pt) is None
        assert not network.nominalDirectionsAt(pt)


def test_orientation_consistency(network):
    for i in range(30):
        pt = network.drivableRegion.uniformPointInner()
        dirs = network.nominalDirectionsAt(pt)
        elem = network.elementAt(pt)
        if isinstance(elem, Intersection):
            assert len(dirs) >= 1
            connectors = set()
            for maneuver in elem.maneuvers:
                cl = maneuver.connectingLane
                if cl.containsPoint(pt):
                    assert pytest.approx(cl.orientation[pt]) in dirs
                    connectors.add(cl)
            if not connectors:
                # Not on any connecting lane; orientation should be given by
                # whichever one is closest.
                man = min(
                    elem.maneuvers, key=lambda man: man.connectingLane.distanceTo(pt)
                )
                assert pytest.approx(man.connectingLane.orientation[pt]) in dirs
            else:
                lane = network.laneAt(pt)
                assert lane in connectors
        else:
            assert len(dirs) == 1
            d = dirs[0]
            road = network.roadAt(pt)
            assert road is elem
            assert road.orientation[pt] == pytest.approx(d)
            roadSec = road.sectionAt(pt)
            assert roadSec.orientation[pt] == pytest.approx(d)
            group = network.laneGroupAt(pt)
            assert road.laneGroupAt(pt) is group
            assert group.orientation[pt] == pytest.approx(d)
            lane = network.laneAt(pt)
            assert road.laneAt(pt) is lane
            assert lane.orientation[pt] == pytest.approx(d)
            laneSec = network.laneSectionAt(pt)
            assert road.laneSectionAt(pt) is laneSec
            assert laneSec.orientation[pt] == pytest.approx(d)


def test_linkage(network):
    for road in network.roads:
        assert road.forwardLanes or road.backwardLanes
        assert road.is1Way == (not (road.forwardLanes and road.backwardLanes))
        seenLanes = set()

        def checkGroup(group):
            assert group.road is road
            if group._sidewalk:
                sidewalk = group._sidewalk
                assert group.sidewalk is group._sidewalk
                assert sidewalk.road is road
                for crossing in sidewalk.crossings:
                    assert crossing.parent is road
                    assert sidewalk in (crossing.startSidewalk, crossing.endSidewalk)
            else:
                with pytest.raises(RejectionException):
                    group.sidewalk
            if group._shoulder:
                shoulder = group._shoulder
                assert group.shoulder is shoulder
                assert shoulder.road is road
            else:
                with pytest.raises(RejectionException):
                    group.shoulder
            if group._bikeLane:
                bikeLane = group._bikeLane
                assert group.bikeLane is bikeLane
                assert bikeLane.road is road
            else:
                with pytest.raises(RejectionException):
                    group.bikeLane
            for lane in group.lanes:
                assert lane not in seenLanes
                seenLanes.add(lane)
                assert lane.group is group
                assert lane.road is road
                for section in lane.sections:
                    assert section.lane is lane
                    assert section.group is group
                    assert section.road is road
                    assert section.isForward == (group is road.forwardLanes)
                    for i in range(30):
                        pt = section.uniformPointInner()
                        assert lane.sectionAt(pt) is section
                    fastSlow = (section._fasterLane, section._slowerLane)
                    leftRight = (section._laneToLeft, section._laneToRight)
                    if section._fasterLane:
                        assert section.fasterLane is section._fasterLane
                        assert section.fasterLane in leftRight
                    if section._slowerLane:
                        assert section.slowerLane is section._slowerLane
                        assert section.slowerLane in leftRight
                    if section._laneToLeft:
                        left = section._laneToLeft
                        assert section.laneToLeft is left
                        assert section.shiftedBy(1) is left
                        assert left is not section._laneToRight
                        if section.isForward == left.isForward:
                            assert fastSlow.count(left) == 1
                            assert left.laneToRight is section
                            assert left.shiftedBy(-1) is section
                        else:
                            assert left.laneToLeft is section
                            assert left.shiftedBy(1) is section
                    else:
                        with pytest.raises(RejectionException):
                            section.laneToLeft
                    if section._laneToRight:
                        right = section._laneToRight
                        assert section.laneToRight is right
                        assert section.shiftedBy(-1) is right
                        assert right is not section._laneToLeft
                        if section.isForward == right.isForward:
                            assert fastSlow.count(right) == 1
                            assert right.laneToLeft is section
                            assert right.shiftedBy(1) is section
                        else:
                            assert right.laneToRight is section
                            assert right.shiftedBy(-1) is section
                    else:
                        with pytest.raises(RejectionException):
                            section.laneToRight
                for maneuver in lane.maneuvers:
                    assert maneuver.startLane is lane
                for i in range(30):
                    pt = lane.uniformPointInner()
                    assert group.laneAt(pt) is lane

        if road.forwardLanes:
            checkGroup(road.forwardLanes)
            assert road.forwardLanes._opposite is road.backwardLanes
            if road.backwardLanes:
                assert road.forwardLanes.opposite is road.backwardLanes
        else:
            with pytest.raises(RejectionException):
                road.backwardLanes.opposite
        if road.backwardLanes:
            checkGroup(road.backwardLanes)
            assert road.backwardLanes._opposite is road.forwardLanes
            if road.forwardLanes:
                assert road.backwardLanes.opposite is road.forwardLanes
        else:
            with pytest.raises(RejectionException):
                road.forwardLanes.opposite
        assert set(road.lanes) == seenLanes

    for intersection in network.intersections:
        allManeuvers = intersection.maneuvers
        allConnecting = set(man.connectingLane for man in allManeuvers)
        for incoming in intersection.incomingLanes:
            assert incoming.road in intersection.roads
            assert incoming.successor in allConnecting
            maneuvers = incoming.maneuvers
            for maneuver in maneuvers:
                assert maneuver in allManeuvers
                assert maneuver.startLane is incoming
                assert maneuver.intersection is intersection
                connecting = maneuver.connectingLane
                assert connecting is not None
                assert connecting in allConnecting
                outgoing = connecting.successor
                assert outgoing in intersection.outgoingLanes
                assert outgoing.road in intersection.roads
                for conf in maneuver.conflictingManeuvers:
                    assert conf is not maneuver
                    assert connecting.intersects(conf.connectingLane)
                for rev in maneuver.reverseManeuvers:
                    assert rev is not maneuver
                    assert rev.startLane.road is maneuver.endLane.road
                    assert rev.endLane.road is maneuver.startLane.road
