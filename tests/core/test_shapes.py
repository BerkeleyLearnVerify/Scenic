import math
from pathlib import Path

import pytest

from scenic.core.shapes import BoxShape, MeshShape


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
