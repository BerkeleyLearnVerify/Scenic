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
from scenic.simulators.isaac import TerrainBase
from scenic.simulators.isaac.simulator import IsaacSimulator
from scenic.simulators.isaac.utils import _addExistingObj, EnvironmentMeshCache

# ---------- global parameters ----------

param environmentUSDPath = None
param environmentInfoPath = None
param environmentMeshPath = None
param headless = False
param isaacBackend = DEFAULT_BACKEND_NAME
param isaacLab = False
environmentMeshPath = globalParameters.environmentMeshPath
environmentInfoPath = globalParameters.environmentInfoPath
set_default_backend(globalParameters.isaacBackend)
isaac_backend = get_backend("lab" if globalParameters.isaacLab else globalParameters.isaacBackend)

param labEnvCfg = None
param labDevice = "cuda:0"
param labNumEnvs = 1
param labEnvSpacing = 10.0
param labTimestep = 1 / 120

param labTask = None
param labDebugLifecycle = False

# ---------- simulator creation ----------

if isaac_backend.name == 'lab':
    simulator IsaacSimulator(
        environmentUSDPath=globalParameters.environmentUSDPath,
        headless=globalParameters.headless,
        backend=globalParameters.isaacBackend,
        isaacLab=globalParameters.isaacLab,
        task = globalParameters.labTask,
        env_cfg_entry_point=globalParameters.labEnvCfg,
        device=globalParameters.labDevice,
        num_envs=globalParameters.labNumEnvs,
        env_spacing=globalParameters.labEnvSpacing,
        timestep=globalParameters.labTimestep,
        debug_lifecycle=globalParameters.labDebugLifecycle,
    )
else:
    simulator IsaacSimulator(
        environmentUSDPath=globalParameters.environmentUSDPath,
        headless=globalParameters.headless,
        backend=globalParameters.isaacBackend,
        isaacLab=globalParameters.isaacLab,
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

    name: f"Robot_{uuid.uuid4().hex[:8]}"
    controller: None
    control: None
    usd_path: None
    isaac_asset_path: None
    initial_rotation: None
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
    isaac_asset_path: "Isaac/Robots/iRobot/Create3/create_3.usd"

    # Differential-drive metadata.
    wheel_radius: 0.03575
    wheel_base: 0.233
    wheel_dof_names: ["left_wheel_joint", "right_wheel_joint"]
    wheel_controller: "differential"

class Jetbot(IsaacSimRobot, _WheeledRobot):
    width: 0.16
    length: 0.16
    height: 0.12
    isaac_asset_path: "Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
   
    # Differential-drive metadata.
    wheel_radius: 0.03 
    wheel_base: 0.1125
    wheel_dof_names: ["left_wheel_joint", "right_wheel_joint"]
    wheel_controller: "differential"

class Kaya(IsaacSimRobot, _HolonomicRobot):

    width: 0.2
    length: 0.2
    height: 0.2
    isaac_asset_path: "Isaac/Robots/NVIDIA/Kaya/kaya.usd"

    # Holonomic-drive metadata.
    wheel_dof_names: ["axle_0_joint", "axle_1_joint", "axle_2_joint"]
    wheel_controller: "holonomic"

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
    end_effector_offset: [0.0, 0.0, 0.0]
    end_effector_orientation: None
    franka_pick_place_events_dt: [60, 40, 20, 40, 80, 20, 20]

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
    
    name: "GroundPlane"
    width: 5
    length: 5
    height: 0.01
    physics: False
    blueprint: "GroundPlane"

    def create(self):
        return isaac_backend.create_ground_plane(self)

class Terrain:
    horizontal_scale: TerrainBase.horizontal_scale
    vertical_scale: TerrainBase.vertical_scale
    width: 10.0
    length: 10.0
    size: (self.width, self.length)
    difficulty: None
    blueprint: "Terrain"
    physics: False
    position: (0, 0, 0)

    # These will be filled by subclasses in create()
    mesh: None
    subterrain: None

class RandomUniformTerrain(Terrain):
    name: f"RandomUniformTerrain_{uuid.uuid4().hex[:8]}"

    noise_range: (0.0, 1.0)
    noise_step: 0.01
    downsampled_scale: 1.0

    def create(self):
        from scenic.core.terrain import random_uniform_terrain, RandomUniformTerrainCfg, subterrain_to_mesh
        # Build configuration and register the generator function for the simulator.
        terrain_cfg = RandomUniformTerrainCfg(
            noise_range=self.noise_range,
            noise_step=self.noise_step,
            downsampled_scale=self.downsampled_scale,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
            border_width=1.0,
            size=self.size,
        )
        subterrain = random_uniform_terrain(cfg=terrain_cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(subterrain)
        self.subterrain = subterrain

class SlopedTerrain(Terrain):
    name: f"SlopedTerrain_{uuid.uuid4().hex[:8]}"
    slope: 0.25

    def create(self):
        from scenic.core.terrain import sloped_terrain, SlopedTerrainCfg, subterrain_to_mesh
        terrain_cfg = SlopedTerrainCfg(
            slope=self.slope,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
            size=self.size,
        )
        subterrain = sloped_terrain(cfg=terrain_cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(subterrain)
        self.subterrain = subterrain

class PyramidSlopedTerrain(Terrain):
    name: f"PyramidSlopedTerrain_{uuid.uuid4().hex[:8]}"
    slope: 0.25
    platform_size: 1.0

    def create(self):
        from scenic.core.terrain import pyramid_sloped_terrain, PyramidSlopedTerrainCfg, subterrain_to_mesh
        terrain_cfg = PyramidSlopedTerrainCfg(
            slope=self.slope,
            platform_size=self.platform_size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
            size=self.size,
        )
        subterrain = pyramid_sloped_terrain(cfg=terrain_cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(subterrain)
        self.subterrain = subterrain

class DiscreteObstaclesTerrain(Terrain):
    name: f"DiscreteObstaclesTerrain_{uuid.uuid4().hex[:8]}"
    max_height: 0.2
    min_size: 0.5
    max_size: 2.0
    num_rects: 5
    platform_size: 1.0

    def create(self):
        from scenic.core.terrain import discrete_obstacles_terrain, DiscreteObstaclesTerrainCfg, subterrain_to_mesh
        terrain_cfg = DiscreteObstaclesTerrainCfg(
            max_height=self.max_height,
            min_size=self.min_size,
            max_size=self.max_size,
            num_rects=self.num_rects,
            platform_size=self.platform_size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
            size=self.size,
        )
        subterrain = discrete_obstacles_terrain(cfg=terrain_cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(subterrain)
        self.subterrain = subterrain

class WaveTerrain(Terrain):
    name: f"WaveTerrain_{uuid.uuid4().hex[:8]}"
    num_waves: 1
    amplitude: 1.0

    def create(self):
        from scenic.core.terrain import wave_terrain, WaveTerrainCfg, subterrain_to_mesh
        cfg = WaveTerrainCfg(
            num_waves=self.num_waves,
            amplitude=self.amplitude,
            size=self.size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
        )
        sub = wave_terrain(cfg=cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(sub)
        self.subterrain = sub

class StairsTerrain(Terrain):
    name: f"StairsTerrain_{uuid.uuid4().hex[:8]}"
    step_width: 1.0
    step_height: 0.1

    def create(self):
        from scenic.core.terrain import stairs_terrain, StairsTerrainCfg, subterrain_to_mesh
        cfg = StairsTerrainCfg(
            step_width=self.step_width,
            step_height=self.step_height,
            size=self.size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
        )
        sub = stairs_terrain(cfg=cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(sub)
        self.subterrain = sub

class PyramidStairsTerrain(Terrain):
    name: f"PyramidStairsTerrain_{uuid.uuid4().hex[:8]}"
    step_width: 1.0
    step_height: 0.1
    platform_size: 1.0

    def create(self):
        from scenic.core.terrain import pyramid_stairs_terrain, PyramidStairsTerrainCfg, subterrain_to_mesh
        cfg = PyramidStairsTerrainCfg(
            step_width=self.step_width,
            step_height=self.step_height,
            platform_size=self.platform_size,
            size=self.size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
        )
        sub = pyramid_stairs_terrain(cfg=cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(sub)
        self.subterrain = sub

class SteppingStonesTerrain(Terrain):
    name: f"SteppingStonesTerrain_{uuid.uuid4().hex[:8]}"
    stone_size: 1.0
    stone_distance: 1.0
    max_height: 0.2
    platform_size: 1.0
    depth: -10.0

    def create(self):
        from scenic.core.terrain import stepping_stones_terrain, SteppingStonesTerrainCfg, subterrain_to_mesh
        cfg = SteppingStonesTerrainCfg(
            stone_size=self.stone_size,
            stone_distance=self.stone_distance,
            max_height=self.max_height,
            platform_size=self.platform_size,
            depth=self.depth,
            size=self.size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
        )
        sub = stepping_stones_terrain(cfg=cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(sub)
        self.subterrain = sub

class PolesTerrain(Terrain):
    name: f"PolesTerrain_{uuid.uuid4().hex[:8]}"
    difficulty: 1.0

    def create(self):
        from scenic.core.terrain import poles_terrain, PolesTerrainCfg, subterrain_to_mesh
        cfg = PolesTerrainCfg(
            difficulty=self.difficulty,
            size=self.size,
            horizontal_scale=self.horizontal_scale,
            vertical_scale=self.vertical_scale,
        )
        sub = poles_terrain(cfg=cfg, difficulty=self.difficulty)
        self.mesh = subterrain_to_mesh(sub)
        self.subterrain = sub

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
                dimensions = tuple(np.maximum(float(x), 1e-6) for x in info["usd_dimensions"])
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
