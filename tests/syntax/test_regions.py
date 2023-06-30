import math

import pytest

import scenic
from scenic.core.errors import InvalidScenarioError
from scenic.core.vectors import Vector
from tests.utils import (
    compileScenic,
    sampleEgo,
    sampleEgoFrom,
    sampleScene,
    sampleSceneFrom,
)

# Built-in regions


def test_everywhere():
    scenario = compileScenic("ego = new Object with regionContainedIn everywhere")
    sampleScene(scenario, maxIterations=1)


def test_nowhere():
    with pytest.raises(InvalidScenarioError):
        compileScenic("ego = new Object with regionContainedIn nowhere")


# CircularRegion


def test_circular_in():
    scenario = compileScenic("ego = new Object in CircularRegion(3@5, 2)")
    positions = [sampleEgo(scenario).position for i in range(30)]
    center = Vector(3, 5)
    distances = [pos.distanceTo(center) for pos in positions]
    assert all(dist <= 2 for dist in distances)
    assert any(dist < 1.5 for dist in distances)
    assert any(dist > 1.5 for dist in distances)
    assert any(pos.x < 3 for pos in positions)
    assert any(pos.x > 3 for pos in positions)


def test_circular_lazy():
    ego = sampleEgoFrom(
        """
        vf = VectorField("Foo", lambda pos: 2 * pos.x)
        x = 0 relative to vf
        ego = new Object at Range(0, 1) @ 0, with foo CircularRegion(0@0, x.yaw)
    """
    )
    assert ego.foo.radius == pytest.approx(2 * ego.position.x)


# SectorRegion


def test_sector_in():
    scenario = compileScenic("ego = new Object in SectorRegion(3@5, 2, -90 deg, 180 deg)")
    positions = [sampleEgo(scenario).position for i in range(50)]
    center = Vector(3, 5)
    distances = [pos.distanceTo(center) for pos in positions]
    assert all(dist <= 2 for dist in distances)
    assert any(dist < 1.5 for dist in distances)
    assert any(dist > 1.5 for dist in distances)
    assert all(pos.x >= 3 for pos in positions)
    assert any(pos.y < 5 for pos in positions)
    assert any(pos.y > 5 for pos in positions)


def test_sector_lazy():
    ego = sampleEgoFrom(
        """
        vf = VectorField("Foo", lambda pos: 2 * pos.x)
        x = 0 relative to vf
        ego = new Object at Range(0, 1) @ 0, with foo SectorRegion(0@0, x.yaw, 0, 45 deg)
    """
    )
    assert ego.foo.radius == pytest.approx(2 * ego.position.x)


# RectangularRegion


def test_rectangular_in():
    scenario = compileScenic("ego = new Object in RectangularRegion(3@5, 45 deg, 3, 8)")
    center = Vector(3, 5)
    offsets = [
        (sampleEgo(scenario).position - center).rotatedBy(math.radians(-45))
        for i in range(50)
    ]
    assert all(-1.5 <= off.x <= 1.5 for off in offsets)
    assert all(-4 <= off.y <= 4 for off in offsets)
    assert any(off.x > 0 for off in offsets)
    assert any(off.x < 0 for off in offsets)
    assert any(off.y > 0 for off in offsets)
    assert any(off.y < 0 for off in offsets)


def test_rectangular_lazy():
    ego = sampleEgoFrom(
        """
        vf = VectorField("Foo", lambda pos: 2 * pos.x)
        x = 0 relative to vf
        ego = new Object at Range(-1, 1) @ 0, with foo RectangularRegion(0@0, 0, x.yaw, 1)
    """
    )
    assert ego.foo.width == pytest.approx(2 * ego.position.x)


# PolygonalRegion


def test_polygonal_empty_intersection():
    scenario = compileScenic(
        """
        r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])
        ego = new Object at -10@0, facing Range(-90, 0) deg, with viewAngle 60 deg
        new Object in visible r1, with requireVisible False
    """
    )
    for i in range(10):
        sampleScene(scenario, maxIterations=1000)


# PolylineRegion


def test_polyline_start():
    ego = sampleEgoFrom(
        """
        r = PolylineRegion([1@1, 3@-1, 6@2])
        pt = r.start
        ego = new Object at pt, facing pt.heading
    """
    )
    assert tuple(ego.position) == pytest.approx((1, 1, 0))
    assert ego.heading == pytest.approx(math.radians(-135))


def test_polyline_end():
    ego = sampleEgoFrom(
        """
        r = PolylineRegion([1@1, 3@-1, 6@2])
        pt = r.end
        ego = new Object at pt, facing pt.heading
    """
    )
    assert tuple(ego.position) == pytest.approx((6, 2, 0))
    assert ego.heading == pytest.approx(math.radians(-45))


# Mesh Regions
def test_mesh_region_distribution():
    sampleSceneFrom(
        """
        position = (Range(-5,5), Range(-5,5), Range(-5,5))
        radius = Range(1,5)
        dimensions = (2*radius, 2*radius, 2*radius)
        rotation = (Range(0,360), Range(0,360), Range(0,360))

        region = SpheroidRegion(position=position, dimensions=dimensions, rotation=rotation)

        ego = new Object in region
    """,
        maxIterations=100,
    )


# View Regions
def test_view_region_construction():
    sampleSceneFrom(
        """
        workspace_region = RectangularRegion(0 @ 0, 0, 40, 40)
        workspace = Workspace(workspace_region)

        sample_space = BoxRegion(dimensions=(30,30,30), position=(0,0,15))

        ego = new Object with visibleDistance 20,
            with width 5,
            with length 5,
            with height 5,
            with viewAngles (360 deg, 180 deg)

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (360 deg, 90 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with viewAngles (180 deg, 180 deg),
            with requireVisible True,
            with cameraOffset (0,0,0.5)

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (180 deg, 90 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (200 deg, 180 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (20 deg, 180 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (90 deg, 90 deg),
            with requireVisible True

        new Object in sample_space,
            with width 1,
            with length 1,
            with height 1,
            facing (Range(0,360) deg, Range(0,360) deg, Range(0,360) deg),
            with visibleDistance 5,
            with viewAngles (200 deg, 40 deg),
            with requireVisible True
    """,
        maxIterations=1000,
    )


# Workspaces


def test_workspace():
    ego = sampleEgoFrom(
        """
        workspace = Workspace(RectangularRegion(5@10, 0, 2, 2))
        ego = new Object in workspace
        require 6@11 in workspace
        require ego in workspace
    """
    )
    assert 3 <= ego.position.x <= 7
    assert 8 <= ego.position.y <= 12
