import os
import tempfile
from urllib.parse import urlparse

import trimesh

from scenic.core.regions import MeshVolumeRegion
from scenic.core.simulators import Simulation, SimulationCreationError, Simulator
from scenic.core.vectors import Vector
from scenic.simulators.isaac.backends import getBackend
import scenic.simulators.isaac.utils as utils


class IsaacSimulator(Simulator):
    def __init__(self, isaacLab=False, **kwargs):
        super().__init__()
        self.isaacLab = isaacLab
        if isaacLab:
            from scenic.simulators.isaac.lab import IsaacLabSimulator

            self.delegate = IsaacLabSimulator(**kwargs)
        else:
            self.delegate = IsaacSimSimulator(**kwargs)

    def createSimulation(self, scene, **kwargs):
        return self.delegate.createSimulation(scene, **kwargs)

    def simulate(self, scene, *args, **kwargs):
        return self.delegate.simulate(scene, *args, **kwargs)

    def destroy(self):
        super().destroy()
        self.delegate.destroy()


class IsaacSimSimulator(Simulator):
    def __init__(self, headless=False, environmentUSDPath=None, backend=None):
        super().__init__()

        self.backend = getBackend(backend)
        self.client = self.backend.getSimulationApp(headless=headless)
        self.environmentUSDPath = environmentUSDPath
        self.headless = headless

    def createSimulation(self, scene, **kwargs):
        return IsaacSimSimulation(
            scene,
            self.client,
            self.environmentUSDPath,
            self.backend,
            headless=self.headless,
            **kwargs,
        )

    def destroy(self):
        super().destroy()
        self.backend.closeSimulationApp(self.client)


class IsaacSimSimulation(Simulation):
    def __init__(
        self,
        scene,
        client,
        environmentUSDPath,
        backend,
        *,
        headless=False,
        timestep,
        **kwargs,
    ):
        self.backend = backend
        self.backend.enableExtension("omni.kit.asset_converter")

        timestep = 1.0 / 60.0 if timestep is None else timestep
        self.client = client
        self.environmentUSDPath = environmentUSDPath
        self.headless = headless
        self.world = None
        self.tmpMeshDir = tempfile.mkdtemp()

        if self.environmentUSDPath:
            self.loadEnvironmentStage()

        self.world = self.backend.createWorld(timestep)

        super().__init__(scene, timestep=timestep, **kwargs)

    def environmentUsdPath(self):
        source = os.fspath(self.environmentUSDPath)
        if source.startswith("Isaac/"):
            return self.backend.assetPath(source)
        if urlparse(source).scheme:
            return source
        return os.path.abspath(source)

    def loadEnvironmentStage(self):
        usd_path = self.environmentUsdPath()

        if not urlparse(usd_path).scheme:
            if not os.path.isfile(usd_path):
                raise SimulationCreationError(
                    f"Isaac Sim environment USD does not exist or is not a file: {usd_path!r}"
                )
            if not os.access(usd_path, os.R_OK):
                raise SimulationCreationError(
                    f"Isaac Sim environment USD is not readable: {usd_path!r}"
                )

        try:
            opened = self.backend.openEnvironmentStage(usd_path)
        except Exception as exc:
            raise SimulationCreationError(
                f"Unable to open Isaac Sim environment USD {usd_path!r}"
            ) from exc

        if not opened:
            raise SimulationCreationError(
                f"Isaac Sim failed to open environment USD {usd_path!r}"
            )

    def setup(self):
        super().setup()

        self.backend.setupLighting(self.headless)
        self.backend.updateApp(self.client)
        self.backend.initializePhysics(self.world, self.objects)
        self.backend.updateApp(self.client)

        from scenic.simulators.isaac.utils import setCollidersExistingObj

        # set collision approximation for all existing environment objects to None
        # setting approximation to None means the mesh geometry is used as the collider
        # according to the USD schema
        setCollidersExistingObj(verbose=True)

        self.backend.playWorld(self.world)

    def step(self):
        self.backend.stepWorld(self.world)

    def createObjectInSimulator(self, obj):
        if (
            obj.blueprint == "IsaacSimObject"
            and not obj.usdPath
            and not obj.isaacAssetPath
        ):
            objectScaledMesh = MeshVolumeRegion(
                mesh=obj.shape.mesh,
                dimensions=(obj.width, obj.length, obj.height),
            ).mesh
            objectObjMesh = utils.meshToObjFrame(objectScaledMesh)
            obj_file_path = os.path.join(self.tmpMeshDir, f"{obj.name}.obj")
            usd_file_path = os.path.join(self.tmpMeshDir, f"{obj.name}.usd")
            trimesh.exchange.export.export_mesh(objectObjMesh, obj_file_path)
            success = self.backend.convertSync(
                obj_file_path, usd_file_path, load_materials=True
            )
            if not success:
                raise SimulationCreationError(
                    f"Unable to convert the mesh for {obj.name} into a USD asset"
                )
            obj.usdPath = usd_file_path

        isaac_sim_obj = obj.create()
        if isaac_sim_obj is None:
            return

        try:
            self.backend.addObject(self.world, isaac_sim_obj, scenic_obj=obj)
        except Exception as exc:
            raise SimulationCreationError(f"Unable to add {obj.name} to world") from exc

    def getProperties(self, obj, properties):
        if not obj.physics:
            return {prop: getattr(obj, prop) for prop in properties}

        raw = self.backend.getPhysicsProperties(self.world, obj)
        return dict(
            position=Vector(*raw["position"]),
            velocity=Vector(*raw["velocity"]),
            speed=raw["speed"],
            angularSpeed=raw["angularSpeed"],
            angularVelocity=Vector(*raw["angularVelocity"]),
            yaw=raw["yaw"],
            pitch=raw["pitch"],
            roll=raw["roll"],
        )

    def destroy(self):
        if self.world is not None:
            self.backend.stopAndClearWorld(self.world)
            self.backend.releaseWorld(self.world)
            self.world = None
