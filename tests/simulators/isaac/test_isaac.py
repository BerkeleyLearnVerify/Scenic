"""Tests for the Isaac Sim interface that do not require Isaac Sim itself.

The Isaac world model can be compiled without Isaac Sim installed, since the
simulator is only instantiated when a simulation is run.
"""

import bz2

import pytest
import trimesh

from tests.utils import compileScenic, pickle_test, sampleScene, tryPickling


def test_backend_registry():
    from scenic.simulators.isaac.backends import (
        DEFAULT_BACKEND_NAME,
        articulationAction,
        detectBackend,
        getBackend,
    )

    backend = getBackend(DEFAULT_BACKEND_NAME)
    assert backend is getBackend(None)
    assert backend.name == detectBackend()
    assert getBackend("core_51").name == "core_51"
    assert getBackend("lab").name == "lab"
    with pytest.raises(ValueError):
        getBackend("nonexistent")

    action = articulationAction(joint_velocities=[1, 2], joint_indices=[0, 1])
    assert action == {"joint_velocities": [1, 2], "joint_indices": [0, 1]}
    assert backend.articulationAction(joint_efforts=[3]) == {"joint_efforts": [3]}


def test_orientation_conversion():
    from scenic.core.vectors import Orientation
    from scenic.simulators.isaac.backends import getBackend

    backend = getBackend("core_51")
    for angles in ((0, 0, 0), (0.3, -0.2, 0.7), (2.0, 1.0, -2.5)):
        orientation = Orientation.fromEuler(*angles)
        quat = backend.scenicToIsaacOrientation(orientation)
        assert quat.shape == (4,)
        yaw, pitch, roll = backend.isaacQuatToScenicEulerAngles(quat)
        assert Orientation.fromEuler(yaw, pitch, roll).approxEq(orientation)

        # initial_rotation is applied first, in the asset's frame.
        initial = (0.5, 0.0, 0.0)
        composed = backend.scenicToIsaacOrientation(orientation, initial_rotation=initial)
        expected = orientation * Orientation.fromEuler(*initial)
        assert Orientation.fromEuler(
            *backend.isaacQuatToScenicEulerAngles(composed)
        ).approxEq(expected)


def test_compressed_paths(tmp_path, monkeypatch):
    from scenic.simulators.isaac import utils

    monkeypatch.setattr(utils.Path, "home", lambda: tmp_path / "home")

    assert utils.assetStem("a/b/simple_room.usd.bz2") == "simple_room"
    assert utils.assetStem("Isaac/Props/KLT_Bin/small_KLT.usd") == "small_KLT"
    mesh_path, info_path = utils.defaultEnvironmentMeshPaths(tmp_path / "room.usd.bz2")
    assert mesh_path == tmp_path / "_converted" / "room_usd.glb.bz2"
    assert info_path == tmp_path / "_converted" / "room_info.json"
    mesh_path, _ = utils.defaultEnvironmentMeshPaths("Isaac/Environments/x/y.usd")
    assert mesh_path.name == "y_usd.glb.bz2" and tmp_path / "home" in mesh_path.parents

    plain = tmp_path / "asset.usda"
    plain.write_text("#usda 1.0\n")
    assert utils.decompressedPath(plain) == plain
    compressed = utils.compressFile(plain, remove_source=True)
    assert compressed == tmp_path / "asset.usda.bz2" and not plain.exists()
    decompressed = utils.decompressedPath(compressed)
    assert decompressed.name == "asset.usda" and decompressed.read_text() == "#usda 1.0\n"
    assert utils.decompressedPath(compressed) == decompressed  # cached


def test_compressed_meshes(tmp_path):
    from scenic.simulators.isaac import utils

    scene = trimesh.Scene()
    scene.add_geometry(trimesh.creation.box((1, 2, 3)), node_name="prim_0")
    scene.add_geometry(
        trimesh.creation.icosphere(),
        node_name="prim_1",
        transform=trimesh.transformations.translation_matrix([5, 0, 0]),
    )
    path = utils.writeMesh(scene, tmp_path / "scene_usd.glb.bz2")
    with bz2.open(path, "rb") as in_file:
        assert in_file.read(4) == b"glTF"

    loaded = trimesh.load(path, force="scene")
    assert sorted(loaded.graph.nodes_geometry) == ["prim_0", "prim_1"]
    assert loaded.graph.get("prim_1")[0][0, 3] == pytest.approx(5)

    mesh = utils.loadAssetMesh(path)
    assert mesh.is_volume

    assert compileScenic(
        f"""
        model scenic.simulators.isaac.model
        from scenic.simulators.isaac.utils import loadAssetMesh
        ego = new IsaacSimObject with shape MeshShape(loadAssetMesh({str(path)!r}))
        """
    )


def test_basic(loadLocalScenario):
    scenario = loadLocalScenario("basic.scenic")
    scene = sampleScene(scenario, maxIterations=1000)
    assert len(scene.objects) == 5
    assert scene.egoObject.wheelController == "differential"
    robot = next(obj for obj in scene.objects if obj.blueprint == "Robot" and obj.control)
    assert robot.wheelController is None
    assert robot.control([1.0, 0.5]) == {
        "joint_velocities": [0.5, 1.5],
        "joint_indices": [0, 1],
    }


def test_isaac_lab_params():
    scenario = compileScenic(
        """
        param isaacLab = True
        param labNumEnvs = 4
        model scenic.simulators.isaac.model
        floor = new GroundPlane
        ego = new Jetbot on floor, with behavior JetbotDrive
        """
    )
    scene = sampleScene(scenario, maxIterations=100)
    assert scene.params["labNumEnvs"] == 4
    assert scene.egoObject.wheelDofNames == ["left_wheel_joint", "right_wheel_joint"]


@pickle_test
@pytest.mark.slow
def test_pickle(loadLocalScenario):
    scenario = tryPickling(loadLocalScenario("basic.scenic"))
    tryPickling(sampleScene(scenario, maxIterations=1000))
