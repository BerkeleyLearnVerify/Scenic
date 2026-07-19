from pathlib import Path

from scenic.domains.driving.roads import ManeuverType, Network

SIGNAL_MAPS = (
    Path(__file__).resolve().parents[3] / "assets" / "maps" / "demo" / "signals"
)


def load_network(name: str) -> Network:
    path = SIGNAL_MAPS / name
    assert path.is_file(), f"missing demo map {path}"
    return Network.fromFile(path, useCache=False)


def open_drive_id(lane) -> int:
    """OpenDRIVE lane id from a Scenic lane (unique within a lane section)."""
    ids = {sec.openDriveID for sec in lane.sections}
    assert len(ids) == 1, lane
    return next(iter(ids))


def all_maneuvers(network: Network):
    return [m for inter in network.intersections for m in inter.maneuvers]


def assert_bidirectional(maneuver):
    """maneuver.signal and signal.controlledManeuvers stay consistent."""
    assert maneuver.signal is not None
    assert maneuver in maneuver.signal.controlledManeuvers
    for other in maneuver.signal.controlledManeuvers:
        assert other.signal is maneuver.signal


def test_validity_lanes_per_lane_signals():
    """Map 01: two straight lanes, each controlled by its own connecting-road light."""
    network = load_network("01_signal_validity_lanes.xodr")
    mans = all_maneuvers(network)
    assert len(mans) == 2
    assert len(network.intersections) == 1

    by_start = {open_drive_id(m.startLane): m for m in mans}
    assert set(by_start) == {-1, -2}

    man_inner = by_start[-1]
    man_outer = by_start[-2]
    assert man_inner.type is ManeuverType.STRAIGHT
    assert man_outer.type is ManeuverType.STRAIGHT

    assert man_inner.signal is not None
    assert man_outer.signal is not None
    assert man_inner.signal is not man_outer.signal
    assert man_inner.signal.openDriveID == "101"
    assert man_outer.signal.openDriveID == "102"
    assert man_inner.signal.isTrafficLight
    assert man_outer.signal.isTrafficLight

    assert_bidirectional(man_inner)
    assert_bidirectional(man_outer)
    assert man_inner.signal.controlledManeuvers == (man_inner,)
    assert man_outer.signal.controlledManeuvers == (man_outer,)


def test_approach_signal_shared_by_both_lanes():
    """Map 02: no connector signals; one approach light covers both maneuvers."""
    network = load_network("02_signal_on_approach.xodr")
    mans = all_maneuvers(network)
    assert len(mans) == 2

    assert all(m.signal is not None for m in mans)
    assert mans[0].signal is mans[1].signal
    assert mans[0].signal.openDriveID == "201"
    assert mans[0].signal.isTrafficLight

    for man in mans:
        assert_bidirectional(man)
    assert set(mans[0].signal.controlledManeuvers) == set(mans)


def test_t_junction_distinct_signals_per_maneuver():
    """Map 03: one approach lane, straight vs right turn under different lights."""
    network = load_network("03_t_junction_turn_signals.xodr")
    mans = all_maneuvers(network)
    assert len(mans) == 2

    by_type = {m.type: m for m in mans}
    assert set(by_type) == {ManeuverType.STRAIGHT, ManeuverType.RIGHT_TURN}

    # Same start lane for both movements
    assert (
        by_type[ManeuverType.STRAIGHT].startLane
        is by_type[ManeuverType.RIGHT_TURN].startLane
    )
    assert open_drive_id(by_type[ManeuverType.STRAIGHT].startLane) == -1

    straight = by_type[ManeuverType.STRAIGHT]
    right = by_type[ManeuverType.RIGHT_TURN]
    assert straight.signal is not None
    assert right.signal is not None
    assert straight.signal is not right.signal
    assert straight.signal.openDriveID == "301"
    assert right.signal.openDriveID == "302"

    assert_bidirectional(straight)
    assert_bidirectional(right)
    assert straight.signal.controlledManeuvers == (straight,)
    assert right.signal.controlledManeuvers == (right,)


def test_unsignalized_maneuvers_have_no_signal():
    """Map 04: same geometry as 01 but no signals → maneuver.signal is None."""
    network = load_network("04_unsignalized_straight.xodr")
    mans = all_maneuvers(network)
    assert len(mans) == 2
    assert all(m.signal is None for m in mans)

    # No signal should claim these maneuvers
    for road in list(network.roads) + list(network.connectingRoads):
        for sig in road.signals:
            assert sig.controlledManeuvers == ()


def test_four_way_protected_left():
    """Map 05: 4 approaches × (1 left + 2 straight); left uses arrow, straights share circular."""
    network = load_network("05_four_way_protected_left.xodr")
    mans = all_maneuvers(network)
    assert len(network.intersections) == 1
    assert len(mans) == 12

    lefts = [m for m in mans if m.type is ManeuverType.LEFT_TURN]
    straights = [m for m in mans if m.type is ManeuverType.STRAIGHT]
    assert len(lefts) == 4
    assert len(straights) == 8

    left_ids = {m.signal.openDriveID for m in lefts}
    straight_ids = {m.signal.openDriveID for m in straights}
    assert left_ids == {"501", "503", "505", "507"}
    assert straight_ids == {"502", "504", "506", "508"}
    assert left_ids.isdisjoint(straight_ids)

    for m in lefts:
        assert open_drive_id(m.startLane) == -1
        assert_bidirectional(m)
        assert m.signal.controlledManeuvers == (m,)

    # Each circular light controls exactly the two straight lanes of that approach
    for sig_id in straight_ids:
        controlled = [m for m in straights if m.signal.openDriveID == sig_id]
        assert len(controlled) == 2
        assert {open_drive_id(m.startLane) for m in controlled} == {-2, -3}
        for m in controlled:
            assert_bidirectional(m)


def test_maneuver_signal_field_defaults_and_intersection_consistency():
    """Every intersection maneuver either has a consistent signal link or None."""
    for name in (
        "01_signal_validity_lanes.xodr",
        "02_signal_on_approach.xodr",
        "03_t_junction_turn_signals.xodr",
        "04_unsignalized_straight.xodr",
        "05_four_way_protected_left.xodr",
    ):
        network = load_network(name)
        for inter in network.intersections:
            for man in inter.maneuvers:
                assert man.intersection is inter
                if man.signal is None:
                    continue
                assert_bidirectional(man)
                # Controlling signal should be a traffic light on these demos
                assert man.signal.isTrafficLight
                assert man.signal.type == "1000001"
