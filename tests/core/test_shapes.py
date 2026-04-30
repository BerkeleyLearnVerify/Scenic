import math
from pathlib import Path

import pytest

from scenic.core.regions import BoxRegion
from scenic.core.shapes import BoxShape, CylinderShape, MeshShape


def test_shape_fromFile(getAssetPath):
    MeshShape.fromFile(
        getAssetPath("meshes/classic_plane.obj.bz2"),
        dimensions=(20, 20, 10),
        initial_rotation=(math.radians(-90), 0, math.radians(-10)),
    )


@pytest.mark.parametrize("badDim", (0, -1))
def test_invalid_dimension(badDim):
    for dims in ((badDim, 1, 1), (1, badDim, 1), (1, 1, badDim)):
        with pytest.raises(ValueError):
            BoxShape(dimensions=dims)
    with pytest.raises(ValueError):
        BoxShape(scale=badDim)


def test_circumradius():
    s = CylinderShape(dimensions=(3, 1, 17))  # dimensions don't matter
    assert s._circumradius == pytest.approx(math.sqrt(2) / 2)


def test_interiorPoint():
    s = MeshShape(BoxRegion().difference(BoxRegion(dimensions=(0.1, 0.1, 0.1))).mesh)
    pt = s._interiorPoint
    assert all(-0.5 <= coord <= 0.5 for coord in pt)
    assert not all(-0.05 <= coord <= 0.05 for coord in pt)
