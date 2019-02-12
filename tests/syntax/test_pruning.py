
import pytest
import math
import random

from scenic.syntax.translator import InterpreterParseError
from tests.utils import compileScenic, sampleEgo

def test_containment():
    scenario = compileScenic(
        'workspace = Workspace(PolygonalRegion([0@0, 2@0, 2@2, 0@2]))\n'
        'ego = Object in workspace'
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(0.5 <= x <= 1.5 for x in xs)
    assert any(0.5 <= x <= 0.7 or 1.3 <= x <= 1.5 for x in xs)

def test_relative_heading():
    scenario = compileScenic(
        'r1 = PolygonalRegion([0@0, 10@0, 10@10, 0@10])\n'      # First cell: heading 0 deg
        'r2 = PolygonalRegion([20@0, 30@0, 30@10, 20@10])\n'    # Second cell: heading 90 deg
        'vf = PolygonalVectorField("Foo", [[r1.polygons, 0], [r2.polygons, 90 deg]])\n'
        'union = r1.union(r2)\n'
        'ego = Object in union, facing vf\n'    # Objects can be in either cell
        'other = Object in union, facing vf\n'
        'require (relative heading of other) >= 60 deg'     # Forces ego in cell 1, other in cell 2
    )
    # Sampling should only require 1 iteration after pruning
    xs = [sampleEgo(scenario).position.x for i in range(100)]
    assert all(0 <= x <= 10 for x in xs)
    assert any(x > 5 for x in xs)
