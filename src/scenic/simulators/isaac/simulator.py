import os
import tempfile
from urllib.parse import urlparse

import trimesh

from scenic.core.regions import MeshVolumeRegion
from scenic.core.simulators import Simulation, SimulationCreationError, Simulator
from scenic.core.vectors import Vector
from scenic.simulators.isaac.backends import get_backend
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

        self.backend = get_backend(backend)
        self.client = self.backend.get_simulation_app(headless=headless)
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
        print("[IsaacSimSimulator.destroy] CLOSING SIMULATION APP", flush=True)
        import traceback

        traceback.print_stack(limit=20)
        super().destroy()
        self.backend.close_simulation_app(self.client)


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
        self.backend.enable_extension("omni.kit.asset_converter")

        timestep = 1.0 / 60.0 if timestep is None else timestep
        self.client = client
        self.environmentUSDPath = environmentUSDPath
        self.headless = headless
        self.world = None
        self.tmpMeshDir = tempfile.mkdtemp()

        if self.environmentUSDPath:
            self.load_environment_stage()

        self.world = self.backend.create_world(timestep)

        super().__init__(scene, timestep=timestep, **kwargs)

    def environment_usd_path(self):
        source = os.fspath(self.environmentUSDPath)
        if source.startswith("Isaac/"):
            return self.backend.asset_path(source)
        if urlparse(source).scheme:
            return source
        return os.path.abspath(source)

    def load_environment_stage(self):
        usd_path = self.environment_usd_path()

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
            opened = self.backend.open_environment_stage(usd_path)
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

        self.backend.setup_lighting(self.headless)
        self.backend.update_app(self.client)
        self.backend.initialize_physics(self.world, self.objects)
        self.backend.update_app(self.client)

        from scenic.simulators.isaac.utils import setCollidersExistingObj

        # set collision approximation for all existing environment objects to None
        # setting approximation to None means the mesh geometry is used as the collider
        # according to the USD schema
        setCollidersExistingObj(verbose=True)

        self.backend.play_world(self.world)

    def step(self):
        self.backend.step_world(self.world)

    def createObjectInSimulator(self, obj):
        if (
            obj.blueprint == "IsaacSimObject"
            and not obj.usd_path
            and not obj.isaac_asset_path
        ):
            objectScaledMesh = MeshVolumeRegion(
                mesh=obj.shape.mesh,
                dimensions=(obj.width, obj.length, obj.height),
            ).mesh
            objectObjMesh = utils.mesh_to_obj_frame(objectScaledMesh)
            obj_file_path = os.path.join(self.tmpMeshDir, f"{obj.name}.obj")
            usd_file_path = os.path.join(self.tmpMeshDir, f"{obj.name}.usd")
            trimesh.exchange.export.export_mesh(objectObjMesh, obj_file_path)
            success = self.backend.convert_sync(
                obj_file_path, usd_file_path, load_materials=True
            )
            if not success:
                raise SimulationCreationError(
                    f"Unable to convert the mesh for {obj.name} into a USD asset"
                )
            obj.usd_path = usd_file_path

        isaac_sim_obj = obj.create()
        if isaac_sim_obj is None:
            return

        try:
            self.backend.add_object(self.world, isaac_sim_obj, scenic_obj=obj)
        except Exception as exc:
            raise SimulationCreationError(f"Unable to add {obj.name} to world") from exc

    def getProperties(self, obj, properties):
        if not obj.physics:
            return {prop: getattr(obj, prop) for prop in properties}

        raw = self.backend.get_physics_properties(self.world, obj)
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
            self.backend.stop_and_clear_world(self.world)
            self.backend.release_world(self.world)
            self.world = None
