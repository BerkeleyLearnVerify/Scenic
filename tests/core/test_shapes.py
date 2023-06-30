import math
from pathlib import Path

from scenic.core.shapes import MeshShape


def test_shape_fromFile():
    MeshShape.fromFile(
        Path(".").parent.parent.parent / "assets" / "meshes" / "classic_plane.obj.bz2",
        dimensions=(20, 20, 10),
        initial_rotation=(math.radians(-90), 0, math.radians(-10)),
    )
