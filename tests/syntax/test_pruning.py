import math
import random

import pytest

from scenic.core.errors import InconsistentScenarioError
from tests.utils import compileScenic, sampleEgo


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
