from pathlib import Path

import numpy
import pytest
import trimesh

from scenic.core.utils import loadMesh, repairMesh


@pytest.mark.slow
def test_mesh_repair(getAssetPath):
    plane_mesh = loadMesh(
        path=getAssetPath("meshes/classic_plane.obj.bz2"),
        filetype="obj",
        compressed=True,
        binary=False,
    )

    # Test simple fix
    inverted_mesh = plane_mesh.copy()
    inverted_mesh.invert()
    assert not inverted_mesh.is_volume
    inverted_mesh = repairMesh(inverted_mesh, verbose=False)
    assert inverted_mesh.is_volume
    assert numpy.allclose(inverted_mesh.bounds, plane_mesh.bounds)
    assert numpy.allclose(
        numpy.mean(inverted_mesh.bounds, axis=0), numpy.mean(plane_mesh.bounds, axis=0)
    )

    # Test missing faces
    broken_mesh = plane_mesh.copy()
    broken_mesh.faces = broken_mesh.faces[40:]
    assert not broken_mesh.is_volume
    broken_mesh = repairMesh(broken_mesh, verbose=False)
    assert broken_mesh.is_volume
    assert numpy.allclose(broken_mesh.bounds, plane_mesh.bounds)
    assert numpy.allclose(
        numpy.mean(broken_mesh.bounds, axis=0), numpy.mean(plane_mesh.bounds, axis=0)
    )

    # Test a planar mesh, which should raise an error.
    plane = trimesh.Trimesh(
        vertices=[(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)],
        faces=[(0, 1, 2), (0, 2, 3)],
    )

    with pytest.raises(ValueError):
        repairMesh(plane)
