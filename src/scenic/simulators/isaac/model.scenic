import json
import os
import uuid
import numpy as np
import trimesh

from scenic.core.errors import InvalidScenarioError
from scenic.core.utils import repairMesh
from trimesh.transformations import decompose_matrix
from scenic.simulators.isaac.actions import *
from scenic.simulators.isaac.actions import (
    _Robot,
    _WheeledRobot,
    _HolonomicRobot,
    _ManipulatorRobot,
)
from scenic.simulators.isaac.behaviors import *
from scenic.simulators.isaac.backends import DEFAULT_BACKEND_NAME, get_backend, set_default_backend
from scenic.simulators.isaac.simulator import IsaacSimSimulator
from scenic.simulators.isaac.utils import _addExistingObj, EnvironmentMeshCache

# ---------- global parameters ----------

param environmentUSDPath = None
param environmentInfoPath = None
param environmentMeshPath = None
param headless = False
param isaacBackend = DEFAULT_BACKEND_NAME
environmentMeshPath = globalParameters.environmentMeshPath
environmentInfoPath = globalParameters.environmentInfoPath
set_default_backend(globalParameters.isaacBackend)
isaac_backend = get_backend(globalParameters.isaacBackend)

# ---------- simulator creation ----------

simulator IsaacSimSimulator(
    environmentUSDPath=globalParameters.environmentUSDPath,
    headless=globalParameters.headless,
    backend=globalParameters.isaacBackend,
)

# ---------- base classes ----------

class IsaacSimObject:

    name: f"Object_{uuid.uuid4().hex[:8]}"
    physics: True
    mass: None
    density: None
    usd_path: None
    isaac_asset_path: None
    blueprint: "IsaacSimObject"

    def create(self):
        return isaac_backend.create_generic_object(self)

class ExistingIsaacSimObject(IsaacSimObject):
    allowCollisions: True
    blueprint: "ExistingIsaacSimObject"
    physics: False
    prim_path: None

    def create(self):
        return None

class IsaacSimRobot(IsaacSimObject, _Robot):

    controller: None
    control: None
    usd_path: None
    isaac_asset_path: None
    blueprint: "Robot"

    def create(self):
        return isaac_backend.create_robot(self)
    
    def move(self, sim, command):
        sim.backend.apply_robot_control(sim, self, command)

class Create3(IsaacSimRobot, _WheeledRobot):

    shape: CylinderShape()
    width: 0.335
    length: 0.335
    height: .1

    def create(self):
        return isaac_backend.create_create3(self)

    def move(self, sim, throttle=0, steering=0):
        sim.backend.apply_wheeled_control(sim, self, [throttle, steering])

class Kaya(IsaacSimRobot, _HolonomicRobot):

    width: 0.2
    length: 0.2
    height: 0.2

    def create(self):
        return isaac_backend.create_kaya(self)
    
    def move(self, sim, forward_speed=0, lateral_speed=0, yaw_speed=0):
        sim.backend.apply_wheeled_control(
            sim, self, [forward_speed, lateral_speed, yaw_speed]
        )

class FrankaPanda(IsaacSimRobot, _ManipulatorRobot):

    # shape: MeshShape(
    #     repairMesh(
    #         trimesh.load(localPath("../../../../assets/FrankaRobotics/FrankaPanda/franka_usd.gltf")).to_geometry()
    #     )
    # )
    shape: BoxShape()
    width: 0.3
    length: 0.3
    height: 0.9
    end_effector_offset: np.array([0, 0.005, 0])
    end_effector_orientation: None

    def create(self):
        return isaac_backend.create_franka_panda(self)

    def move(
        self,
        sim,
        target_object,
        goal_position,
        end_effector_offset=None,
        end_effector_orientation=None,
    ):
        sim.backend.move_franka_pick_place(
            sim,
            self,
            target_object,
            goal_position,
            end_effector_offset,
            end_effector_orientation,
        )

class GroundPlane(IsaacSimObject):

    width: 5
    length: 5
    height: 0.01
    physics: False
    blueprint: "GroundPlane"

    def create(self):
        return isaac_backend.create_ground_plane(self)

if globalParameters.environmentUSDPath:

    try:
        environmentMeshPath, environmentInfoPath = isaac_backend.ensure_environment_mesh_paths(
            globalParameters.environmentUSDPath,
            environmentMeshPath,
            environmentInfoPath,
            headless=globalParameters.headless,
        )
    except Exception as exc:
        raise InvalidScenarioError(
            f"Unable to prepare Isaac environment mesh assets for "
            f"{globalParameters.environmentUSDPath!r}: {exc}"
        ) from exc

    if not os.path.isfile(environmentMeshPath):
        raise InvalidScenarioError(
            f"Isaac environment mesh file does not exist: {environmentMeshPath}"
        )
    if not os.path.isfile(environmentInfoPath):
        raise InvalidScenarioError(
            f"Isaac environment info file does not exist: {environmentInfoPath}"
        )

    with open(environmentInfoPath, "r") as inFile:
        scene = trimesh.load(environmentMeshPath, force="scene")
        meshData = json.load(inFile)

        if not scene.geometry:
            raise InvalidScenarioError(
                f"Isaac environment mesh file has no geometry: {environmentMeshPath}. "
                "Regenerate the GLTF and make sure any external GLTF buffer files are present."
            )

        geometry_nodes = list(scene.graph.nodes_geometry)
        if not geometry_nodes:
            raise InvalidScenarioError(
                f"Isaac environment mesh file has geometry but no scene graph nodes: "
                f"{environmentMeshPath}"
            )

        environmentMeshCache = EnvironmentMeshCache(environmentMeshPath, environmentInfoPath)

        for node_name in geometry_nodes:
            if node_name not in meshData:
                available = ", ".join(sorted(meshData))
                raise InvalidScenarioError(
                    f"Isaac environment mesh node {node_name!r} is missing from "
                    f"{environmentInfoPath}; available nodes: {available}"
                )

            world_transform, geometry_name = scene.graph.get(node_name, "World")
            if geometry_name is None:
                geometry_name = node_name
            mesh = scene.geometry[geometry_name]
            scale, shear, angles, tr, persp = decompose_matrix(world_transform)

            pitch, roll, yaw = angles
            path = meshData[node_name]["full_path"]

            # Prefer authoritative USD bbox data generated before GLTF conversion/repair.
            info = meshData[node_name]

            if "world_bbox_center" in info:
                world_center = np.array(info["world_bbox_center"], dtype=float)
            else:
                local_center = mesh.bounding_box.centroid
                local_center_homogeneous = np.append(local_center, 1.0)
                world_center = np.dot(world_transform, local_center_homogeneous)[:3]

            if "usd_dimensions" in info:
                dimensions = tuple(float(x) for x in info["usd_dimensions"])
            else:
                scale_vec = np.abs(np.array(scale, dtype=float))
                raw_extents = np.array(mesh.extents, dtype=float)
                dimensions = tuple(np.maximum(raw_extents * scale_vec, 1e-6))

            shape_mesh = environmentMeshCache.get(node_name, mesh)

            newObj = new ExistingIsaacSimObject at world_center,
                        with shape MeshShape(shape_mesh, dimensions=dimensions),
                        with name path,
                        with prim_path path,
                        facing (yaw, pitch, roll)

            _addExistingObj(newObj)

        environmentMeshCache.save()
