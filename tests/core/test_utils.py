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

    # Test triangle soup
    random_soup = trimesh.creation.random_soup(face_count=10)
    assert not random_soup.is_volume
    new_soup = repairMesh(random_soup, verbose=False)
    assert new_soup.is_volume
    assert numpy.allclose(random_soup.bounds, new_soup.bounds)
    assert numpy.allclose(
        numpy.mean(random_soup.bounds, axis=0), numpy.mean(new_soup.bounds, axis=0)
    )
