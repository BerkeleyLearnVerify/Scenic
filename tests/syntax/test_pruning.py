import math
import random

import pytest

from scenic.core.errors import InconsistentScenarioError
from scenic.core.vectors import Vector
from tests.utils import compileScenic, sampleEgo, sampleParamP


def test_containment_in():
    """Test pruning based on object containment."""
    scenario = compileScenic(
        """
        workspace = Workspace(PolygonalRegion([0@0, 2@0, 2@2, 0@2]))
        ego = new Object in workspace
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0.5 <= x <= 1.5 for x in xs)
    assert any(0.5 <= x <= 0.7 or 1.3 <= x <= 1.5 for x in xs)


def test_containment_2d_region():
    """Test pruning based on object containment in a 2D region.

    Specifically tests that vertical portions of baseOffset are not added
    to maxDistance, and that if objects are known to be flat in the plane,
    their height is not considered as part of the minRadius.
    """
    # Tests the effect of the vertical portion of baseOffset in a 2D region.
    scenario = compileScenic(
        """
        workspace = Workspace(PolygonalRegion([0@0, 2@0, 2@2, 0@2]))
        ego = new Object on workspace
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0.5 <= x <= 1.5 for x in xs)
    assert any(0.5 <= x <= 0.7 or 1.3 <= x <= 1.5 for x in xs)

    # Test height's effect in a 2D region.
    scenario = compileScenic(
        """
        workspace = Workspace(PolygonalRegion([0@0, 2@0, 2@2, 0@2]))
        ego = new Object in workspace, with height 0.1
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0.5 <= x <= 1.5 for x in xs)
    assert any(0.5 <= x <= 0.7 or 1.3 <= x <= 1.5 for x in xs)

    # Test both combined, in a slightly more complicated case.
    # Specifically, there is a non vertical component to baseOffset
    # that should be accounted for and the height is random.
    scenario = compileScenic(
        """
        class TestObject:
            baseOffset: (0.1, 0, self.height/2)

        workspace = Workspace(PolygonalRegion([0@0, 2@0, 2@2, 0@2]))
        ego = new TestObject on workspace, with height Range(0.1,0.5)
        """
    )
    # Sampling should fail ~30.56% of the time, so
    # 34 rejections are allowed to get the failure probability
    # to ~1e-18.
    xs = [sampleEgo(scenario, maxIterations=34).position.x for i in range(60)]
    assert all(0.5 <= x <= 1.5 for x in xs)
    assert any(0.5 <= x <= 0.7 or 1.3 <= x <= 1.5 for x in xs)


def test_containment_in_polyline():
    """As above, but when the object is placed on a polyline."""
    scenario = compileScenic(
        """
        workspace = Workspace(PolygonalRegion([0@0, 2@0, 2@2, 0@2]))
        line = PolylineRegion([0@0, 1@1, 2@0])
        ego = new Object in line, facing 0
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0.5 <= x <= 1.5 for x in xs)
    assert any(0.5 <= x <= 0.7 or 1.3 <= x <= 1.5 for x in xs)


def test_relative_heading_require_visible():
    """Test pruning based on requirements bounding relative headings."""
    scenario = compileScenic(
        """
        r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])      # First cell: heading 0 deg
        r2 = PolygonalRegion([20@0, 30@0, 30@10, 20@10])    # Second cell: heading 90 deg
        vf = PolygonalVectorField("Foo", [[r1.polygons, 0], [r2.polygons, 90 deg]])
        union = r1.union(r2)
        ego = new Object in union, facing vf                # Objects can be in either cell
        other = new Object in union, facing vf, with requireVisible True
        require (relative heading of other) >= 60 deg   # Forces ego in cell 1, other in cell 2
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0 <= x <= 10 for x in xs)
    assert any(x > 5 for x in xs)


def test_relative_heading_visible_from():
    """Test pruning based on requirements bounding relative headings."""
    scenario = compileScenic(
        """
        r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])      # First cell: heading 0 deg
        r2 = PolygonalRegion([20@0, 30@0, 30@10, 20@10])    # Second cell: heading 90 deg
        vf = PolygonalVectorField("Foo", [[r1.polygons, 0], [r2.polygons, 90 deg]])
        union = r1.union(r2)
        ego = new Object in union, facing vf                # Objects can be in either cell
        other = new Object in union, facing vf, visible from ego
        require (relative heading of other) >= 60 deg   # Forces ego in cell 1, other in cell 2
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0 <= x <= 10 for x in xs)
    assert any(x > 5 for x in xs)


def test_relative_heading_distance():
    """Variant of the above where a distance bound must be inferred from a requirement."""
    scenario = compileScenic(
        """
        r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])      # First cell: heading 0 deg
        r2 = PolygonalRegion([20@0, 30@0, 30@10, 20@10])    # Second cell: heading 90 deg
        r3 = PolygonalRegion([50@0, 60@0, 60@10, 50@10])    # Third cell: heading 90 deg
        vf = PolygonalVectorField("Foo", [[r1.polygons, 0], [r2.polygons, 90 deg],
                                           [r3.polygons, 90 deg]])
        union = r1.union(r2).union(r3)
        ego = new Object in union, facing vf, with visibleDistance 100
        other = new Object in union, facing vf
        require (relative heading of other) >= 60 deg       # Forces ego in cell 1, other cell 2/3
        require (distance to other) <= 35                   # Forces other in cell 2
        """
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(60)]
    assert all(0 <= x <= 10 for x in xs)
    assert any(x > 5 for x in xs)


def test_relative_heading_inconsistent():
    """A special case where we can detect inconsistency of the requirements."""
    with pytest.raises(InconsistentScenarioError):
        scenario = compileScenic(
            """
            ego = new Object
            other = new Object at 10@10
            require abs(relative heading of other) < -1
        """
        )


def test_visibility_pruning():
    """Test visibility pruning in general.

    The following scenarios are equivalent except for how they specify that foo
    must be visible from ego. The size of the workspace and the visibleDistance
    of ego are chosen such that without pruning the chance of sampling a valid
    scene over 100 tries is 1-(1-Decimal(3.14)/Decimal(1e10**2))**100 = ~1e-18.
    Assuming the approximately buffered volume of the viewRegion has a 50% chance of
    rejecting (i.e. it is twice as large as the true buffered viewRegion, which testing
    indicates in this case has about a 10% increase in volume for this case), the chance
    of not finding a sample in 100 iterations is 1e-31.

    We also want to confirm that we aren't pruning too much, i.e. placing the position
    in the viewRegion instead of at any point where the object intersects the view region.
    Because of this, we want to see at least one sample where the position is outside
    the viewRegion but the object intersects the viewRegion. The chance of this happening
    per sample is 1 - (1 / 1.1)**3 = ~25%, so by repeating the process 30 times we have
    a 1e-19 chance of not getting a single point in this zone.
    """
    # requireVisible
    scenario = compileScenic(
        """
        workspace = Workspace(RectangularRegion(0@0, 0, 1e10, 1e10))
        ego = new Object at (0,0,0), with visibleDistance 1
        foo = new Object in workspace, with requireVisible True,
            with shape SpheroidShape(dimensions=(0.2,0.2,0.2))
        param p = foo.position
        """
    )
    positions = [sampleParamP(scenario, maxIterations=100) for i in range(30)]
    assert all(pos.distanceTo(Vector(0, 0, 0)) <= 1.1 for pos in positions)
    assert any(pos.distanceTo(Vector(0, 0, 0)) >= 1 for pos in positions)

    # visible
    scenario = compileScenic(
        """
        workspace = Workspace(RectangularRegion(0@0, 0, 1e10, 1e10))
        ego = new Object at (0,0,0), with visibleDistance 1
        foo = new Object in workspace, visible,
            with shape SpheroidShape(dimensions=(0.2,0.2,0.2))
        param p = foo.position
        """
    )
    positions = [sampleParamP(scenario, maxIterations=100) for i in range(30)]
    assert all(pos.distanceTo(Vector(0, 0, 0)) <= 1.1 for pos in positions)
    assert any(pos.distanceTo(Vector(0, 0, 0)) >= 1 for pos in positions)

    # A minor variation with a miniscule offset
    scenario = compileScenic(
        """
        workspace = Workspace(RectangularRegion(0@0, 0, 1e10, 1e10))
        ego = new Object at (0,0,0), with visibleDistance 1
        foo = new Object on workspace, visible,
            with shape SpheroidShape(dimensions=(0.2,0.2,0.2)),
            with baseOffset (0,0,0.00000001)
        param p = foo.position
        """
    )
    positions = [sampleParamP(scenario, maxIterations=100) for i in range(30)]
    assert all(pos.distanceTo(Vector(0, 0, 0)) <= 1.1 for pos in positions)
    assert any(pos.distanceTo(Vector(0, 0, 0)) >= 1 for pos in positions)


def test_visibility_pruning_cyclical():
    """A case where a cyclical dependency could be introduced if pruning is not done carefully."""
    scenario = compileScenic(
        """
        workspace = Workspace(PolygonalRegion([0@0, 100@0, 100@100, 0@100]))
        foo = new Object with requireVisible True, in workspace
        ego = new Object visible from foo, in workspace
        """
    )

    sampleEgo(scenario, maxIterations=100)
