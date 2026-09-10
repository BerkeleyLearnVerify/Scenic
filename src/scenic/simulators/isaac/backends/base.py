import asyncio
import atexit
from dataclasses import dataclass
import os

import numpy as np

import scenic.simulators.isaac.utils as scenic_utils


class IsaacBackend:
    """Interface implemented by Isaac Sim API backends."""

    name = None

    def __init__(self):
        self._simulation_app = None

    def _simulationAppConfig(self, headless):
        return {
            "headless": headless,
            "sync_loads": True,
            "fast_shutdown": True,
            "multi_gpu": False,
            "max_gpu_count": 1,
        }

    def _closeSimulationAppAtExit(self):
        if self._simulation_app is not None:
            self._simulation_app.close()
            self._simulation_app = None

    def closeSimulationApp(self, app):
        if app is self._simulation_app:
            app.close()
            self._simulation_app = None
        else:
            app.close()

    def attachSimulationApp(self, app):
        self._simulation_app = app

    def getSimulationApp(self, headless=False):
        if self._simulation_app is None:
            from isaacsim.simulation_app import SimulationApp

            self._simulation_app = SimulationApp(
                launch_config=self._simulationAppConfig(headless)
            )
            atexit.register(self._closeSimulationAppAtExit)
        return self._simulation_app

    def createWorld(self, timestep):
        raise NotImplementedError

    def getAssetsRootPath(self):
        from isaacsim.storage.native import get_assets_root_path

        return get_assets_root_path()

    def assetPath(self, relative_path):
        return f"{self.getAssetsRootPath()}/{relative_path}"

    def openEnvironmentStage(self, usd_path):
        raise NotImplementedError

    def setMeshCollisionApproximation(self, prim_path, approximation):
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if prim is None or not prim.IsValid():
            return

        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision_api.GetApproximationAttr().Set(approximation)

        print(f"[DEBUG] {prim_path} approximation set to {approximation}")

    def enableExtension(self, name):
        raise NotImplementedError

    def setupLighting(self, headless):
        import omni.kit.actions.core

        action = None
        if not headless:
            action_registry = omni.kit.actions.core.get_action_registry()
            action = action_registry.get_action(
                "omni.kit.viewport.menubar.lighting",
                "set_lighting_mode_camera",
            )
        if action is not None:
            action.execute()

    def updateApp(self, app):
        app.update()

    def isStageLoading(self):
        from isaacsim.core.utils.stage import is_stage_loading

        return is_stage_loading()

    def initializePhysics(self, world, objects):
        raise NotImplementedError

    def playWorld(self, world):
        raise NotImplementedError

    def stepWorld(self, world):
        raise NotImplementedError

    def stopAndClearWorld(self, world):
        raise NotImplementedError

    def releaseWorld(self, world):
        pass

    def addObject(self, world, obj):
        pass

    async def convert(self, in_file, out_file, load_materials=False):
        import omni.kit.asset_converter

        def progressCallback(progress, total_steps):
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        converter_context.ignore_materials = not load_materials
        converter_context.ignore_animation = False
        converter_context.ignore_cameras = True
        converter_context.use_meter_as_world_unit = True
        converter_context.create_world_as_default_root_prim = True
        instance = omni.kit.asset_converter.get_instance()
        task = instance.create_converter_task(
            in_file, out_file, progressCallback, converter_context
        )
        while True:
            success = await task.wait_until_finished()
            if success:
                return True
            await asyncio.sleep(0.1)

    def runCoroutine(self, coro):
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        if loop.is_running():
            import omni.kit.app

            task = loop.create_task(coro)
            app = omni.kit.app.get_app()
            while not task.done():
                app.update()
            return task.result()

        return loop.run_until_complete(coro)

    def convertSync(self, in_file, out_file, load_materials=False):
        return self.runCoroutine(
            self.convert(in_file, out_file, load_materials=load_materials)
        )

    def kitAppRunning(self):
        try:
            import omni.kit.app

            return omni.kit.app.get_app() is not None
        except Exception:
            return False

    def kitUsdPath(self, environmentUsdPath):
        source = os.fspath(environmentUsdPath)
        if scenic_utils.isIsaacAssetReference(source):
            return self.assetPath(source)
        if scenic_utils.hasUrlScheme(source):
            return source
        return str(scenic_utils.resolvedPath(source))

    def scenicToIsaacOrientation(self, orientation, initial_rotation=None):
        """Convert a Scenic Orientation to an Isaac Sim wxyz quaternion.

        Scenic Euler convention:
            yaw, pitch, roll = intrinsic Z, X, Y rotations.

        Isaac Sim convention:
            quaternion in scalar-first order: w, x, y, z.
        """
        from scipy.spatial.transform import Rotation as R

        yaw, pitch, roll = orientation.eulerAngles

        scenic_rot = R.from_euler("ZXY", [yaw, pitch, roll], degrees=False)

        if initial_rotation is not None:
            iyaw, ipitch, iroll = initial_rotation
            initial_rot = R.from_euler("ZXY", [iyaw, ipitch, iroll], degrees=False)

            scenic_rot = scenic_rot * initial_rot

        # Scenic uses intrinsic Z-X-Y Euler angles.
        q_xyzw = scenic_rot.as_quat()

        # scipy returns xyzw; Isaac Sim expects wxyz.
        q_wxyz = np.array(
            [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]],
            dtype=float,
        )

        norm = np.linalg.norm(q_wxyz)
        if norm == 0:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        return q_wxyz / norm

    def isaacQuatToScenicEulerAngles(self, quat):
        """Convert an Isaac Sim wxyz quaternion to Scenic yaw, pitch, roll.

        Scenic Euler convention:
            yaw, pitch, roll = intrinsic Z, X, Y rotations.

        Isaac Sim convention:
            quaternion in scalar-first order: w, x, y, z.
        """
        from scipy.spatial.transform import Rotation as R

        q_wxyz = np.asarray(quat, dtype=float)

        norm = np.linalg.norm(q_wxyz)
        if norm == 0:
            raise ValueError("cannot convert zero quaternion to Euler angles")

        q_wxyz = q_wxyz / norm

        # Isaac wxyz -> scipy xyzw
        q_xyzw = np.array(
            [q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]],
            dtype=float,
        )

        yaw, pitch, roll = R.from_quat(q_xyzw).as_euler("ZXY", degrees=False)
        return float(yaw), float(pitch), float(roll)

    def computePrimWorldBbox(self, prim_path):
        """Return world-space bbox min, max, center, and size for a prim."""
        from isaacsim.core.utils import prims
        from pxr import Usd, UsdGeom

        prim = prims.get_prim_at_path(prim_path)
        if prim is None or not prim.IsValid():
            raise ValueError(f"invalid prim path: {prim_path}")

        return self.computePrimBbox(prim)

    def computeUsdAssetBbox(self, usd_path):
        """Return the composed bbox of a USD asset referenced at the origin."""
        from pxr import Usd

        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Asset", "Xform")
        prim.GetReferences().AddReference(os.fspath(usd_path))
        return self.computePrimBbox(prim)

    def computePrimBbox(self, prim):
        """Return world-space bbox min, max, center, and size for a USD prim."""
        from pxr import Usd, UsdGeom

        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            useExtentsHint=True,
        )

        # bbox = bbox_cache.ComputeLocalBound(prim)
        # bbox_range = bbox.ComputeAlignedRange()

        # native_min = np.asarray(bbox_range.GetMin(), dtype=float)
        # native_max = np.asarray(bbox_range.GetMax(), dtype=float)
        # native_size = native_max - native_min
        # native_center = (native_min + native_max) / 2.0

        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mn = np.asarray(box.GetMin(), dtype=float)
        mx = np.asarray(box.GetMax(), dtype=float)
        center = (mn + mx) * 0.5
        size = mx - mn

        return mn, mx, center, size

    def rotateVectorByWxyzQuat(self, quat_wxyz, vec):
        """Rotate a vector by an Isaac/Usd wxyz quaternion."""
        from scipy.spatial.transform import Rotation as R

        quat_wxyz = np.asarray(quat_wxyz, dtype=float)
        vec = np.asarray(vec, dtype=float)

        q_xyzw = np.array(
            [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]],
            dtype=float,
        )

        return R.from_quat(q_xyzw).apply(vec)

    def computeUsdScaleAndRootPosition(
        self, obj, prim_path, scenic_position, orientation
    ):
        """Compute local USD scale so the referenced asset matches Scenic dimensions.

        Returns:
            root_position: position to give the USD root prim
            local_scale: x/y/z scale to apply to the USD root prim
            native_size: measured unscaled USD bbox size
            native_center: measured unscaled USD bbox center relative to the root placement
        """
        _, _, native_center, native_size = self.computePrimWorldBbox(prim_path)
        return self.computeScaleAndRootPosition(
            obj,
            native_center,
            native_size,
            scenic_position,
            orientation,
        )

    def computeUsdAssetScaleAndRootPosition(
        self,
        obj,
        usd_path,
        scenic_position,
        orientation,
    ):
        """Compute Scenic scale and root position before a USD asset is spawned."""
        _, _, native_center, native_size = self.computeUsdAssetBbox(usd_path)
        return self.computeScaleAndRootPosition(
            obj,
            native_center,
            native_size,
            scenic_position,
            orientation,
        )

    def computeScaleAndRootPosition(
        self,
        obj,
        native_center,
        native_size,
        scenic_position,
        orientation,
    ):
        """Compute scale and root position from an asset's native bounding box."""
        desired_size = np.array(
            [float(obj.width), float(obj.length), float(obj.height)],
            dtype=float,
        )

        native_center = np.asarray(native_center, dtype=float)
        native_size = np.asarray(native_size, dtype=float)
        local_scale = desired_size / native_size

        # avoid tiny numerical scale changes when dimensions already match.
        if np.allclose(local_scale, np.ones(3), rtol=1e-5, atol=1e-7):
            local_scale = np.ones(3, dtype=float)

        scenic_position = np.asarray(scenic_position, dtype=float)

        # If the asset's geometry center is offset from its root prim,
        # scaling changes that offset. We compensate so the final visual bbox
        # center lands at Scenic's obj.position.
        scaled_center_offset_local = native_center * local_scale
        scaled_center_offset_world = self.rotateVectorByWxyzQuat(
            orientation,
            scaled_center_offset_local,
        )

        root_position = scenic_position - scaled_center_offset_world

        return root_position, local_scale, native_size, native_center

    def createGenericObject(self, obj):
        raise NotImplementedError

    def createRobot(self, obj):
        raise NotImplementedError

    def createWheeledRobot(self, obj):
        raise NotImplementedError

    def createManipulator(self, obj):
        raise NotImplementedError

    def createGroundPlane(self, obj):
        raise NotImplementedError

    def applyRobotControl(self, sim, obj, command):
        raise NotImplementedError

    def applyWheeledControl(self, sim, obj, command):
        raise NotImplementedError

    def applyArticulationAction(self, sim, obj, action):
        raise NotImplementedError

    def articulationDofNames(self, sim, obj):
        raise NotImplementedError

    def articulationDofIndices(self, sim, obj, names):
        dof_names = self.articulationDofNames(sim, obj)
        return [dof_names.index(name) for name in names]

    def getObjectPose(self, sim, obj):
        raise NotImplementedError

    def setObjectPose(self, sim, obj, position, orientation=None):
        raise NotImplementedError

    def moveManipulatorPickPlace(
        self,
        sim,
        obj,
        targetObject,
        goalPosition,
        endEffectorOffset=None,
        endEffectorOrientation=None,
    ):
        raise NotImplementedError

    def moveManipulatorEndEffector(self, sim, obj, position, orientation=None):
        raise NotImplementedError

    def setManipulatorGripper(self, sim, obj, opened):
        raise NotImplementedError

    def setManipulatorArmJointPositions(self, sim, obj, joint_positions):
        raise NotImplementedError

    def holdManipulatorPosition(self, sim, obj):
        raise NotImplementedError

    def getManipulatorEndEffectorPose(self, sim, obj):
        raise NotImplementedError

    def getManipulatorGripperPositions(self, sim, obj):
        raise NotImplementedError

    def manipulatorGripperTargetPositions(self, profile, opened):
        raise NotImplementedError

    def getPhysicsProperties(self, world, obj):
        raise NotImplementedError

    def articulationAction(self, **kwargs):
        return dict(kwargs)
