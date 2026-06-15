import asyncio
import atexit
from dataclasses import dataclass
import os

import numpy as np

import scenic.simulators.isaac.utils as scenic_utils


@dataclass(frozen=True)
class IsaacArticulationAction:
    """Backend-neutral articulation action payload."""

    kwargs: dict


class IsaacBackend:
    """Interface implemented by Isaac Sim API backends."""

    name = None

    def __init__(self):
        self._simulation_app = None

    def _simulation_app_config(self, headless):
        return {
            "headless": headless,
            "sync_loads": True,
            "fast_shutdown": True,
            "multi_gpu": False,
            "max_gpu_count": 1,
        }

    def _close_simulation_app_at_exit(self):
        if self._simulation_app is not None:
            self._simulation_app.close()
            self._simulation_app = None

    def close_simulation_app(self, app):
        if app is self._simulation_app:
            app.close()
            self._simulation_app = None
        else:
            app.close()

    def get_simulation_app(self, headless=False):
        if self._simulation_app is None:
            from isaacsim.simulation_app import SimulationApp

            self._simulation_app = SimulationApp(
                launch_config=self._simulation_app_config(headless)
            )
            atexit.register(self._close_simulation_app_at_exit)
        return self._simulation_app

    def create_world(self, timestep):
        raise NotImplementedError

    def get_assets_root_path(self):
        from isaacsim.storage.native import get_assets_root_path

        return get_assets_root_path()

    def asset_path(self, relative_path):
        return f"{self.get_assets_root_path()}/{relative_path}"

    def open_environment_stage(self, usd_path):
        raise NotImplementedError

    def set_mesh_collision_approximation(self, prim_path, approximation):
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if prim is None or not prim.IsValid():
            return

        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision_api.GetApproximationAttr().Set(approximation)

        print(f"[DEBUG] {prim_path} approximation set to {approximation}")

    def enable_extension(self, name):
        raise NotImplementedError

    def setup_lighting(self, headless):
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

    def update_app(self, app):
        app.update()

    def is_stage_loading(self):
        from isaacsim.core.utils.stage import is_stage_loading

        return is_stage_loading()

    def initialize_physics(self, world, objects):
        raise NotImplementedError

    def play_world(self, world):
        raise NotImplementedError

    def step_world(self, world):
        raise NotImplementedError

    def stop_and_clear_world(self, world):
        raise NotImplementedError

    def release_world(self, world):
        pass

    def add_object(self, world, obj):
        raise NotImplementedError

    async def convert(self, in_file, out_file, load_materials=False):
        import omni.kit.asset_converter

        def progress_callback(progress, total_steps):
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        converter_context.ignore_materials = not load_materials
        converter_context.ignore_animation = False
        converter_context.ignore_cameras = True
        converter_context.use_meter_as_world_unit = True
        converter_context.create_world_as_default_root_prim = True
        instance = omni.kit.asset_converter.get_instance()
        task = instance.create_converter_task(
            in_file, out_file, progress_callback, converter_context
        )
        while True:
            success = await task.wait_until_finished()
            if success:
                return True
            await asyncio.sleep(0.1)

    def run_coroutine(self, coro):
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

    def convert_sync(self, in_file, out_file, load_materials=False):
        return self.run_coroutine(
            self.convert(in_file, out_file, load_materials=load_materials)
        )

    def kit_app_running(self):
        try:
            import omni.kit.app

            return omni.kit.app.get_app() is not None
        except Exception:
            return False

    def kit_usd_path(self, environment_usd_path):
        source = os.fspath(environment_usd_path)
        if scenic_utils.is_isaac_asset_reference(source):
            return self.asset_path(source)
        if scenic_utils.has_url_scheme(source):
            return source
        return str(scenic_utils.resolvedPath(source))

    def scenic_to_isaac_orientation(self, orientation, initial_rotation=None):
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

    def isaac_quat_to_scenic_euler_angles(self, quat):
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

    def _vec3_to_np(self, vec):
        return np.array([float(vec[0]), float(vec[1]), float(vec[2])], dtype=float)

    def compute_prim_world_bbox(self, prim_path):
        """Return world-space bbox min, max, center, and size for a prim."""
        from isaacsim.core.utils import prims
        from pxr import Usd, UsdGeom

        prim = prims.get_prim_at_path(prim_path)
        if prim is None or not prim.IsValid():
            raise ValueError(f"invalid prim path: {prim_path}")

        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            useExtentsHint=True,
        )

        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mn = self._vec3_to_np(box.GetMin())
        mx = self._vec3_to_np(box.GetMax())
        center = (mn + mx) * 0.5
        size = mx - mn

        return mn, mx, center, size

    def rotate_vector_by_wxyz_quat(self, quat_wxyz, vec):
        """Rotate a vector by an Isaac/Usd wxyz quaternion."""
        from scipy.spatial.transform import Rotation as R

        quat_wxyz = np.asarray(quat_wxyz, dtype=float)
        vec = np.asarray(vec, dtype=float)

        q_xyzw = np.array(
            [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]],
            dtype=float,
        )

        return R.from_quat(q_xyzw).apply(vec)

    def compute_usd_scale_and_root_position(
        self, obj, prim_path, scenic_position, orientation
    ):
        """Compute local USD scale so the referenced asset matches Scenic dimensions.

        Returns:
            root_position: position to give the USD root prim
            local_scale: x/y/z scale to apply to the USD root prim
            native_size: measured unscaled USD bbox size
            native_center: measured unscaled USD bbox center relative to the root placement
        """
        _, _, native_center, native_size = self.compute_prim_world_bbox(prim_path)

        desired_size = np.array(
            [float(obj.width), float(obj.length), float(obj.height)],
            dtype=float,
        )

        local_scale = desired_size / native_size

        # avoid tiny numerical scale changes when dimensions already match.
        if np.allclose(local_scale, np.ones(3), rtol=1e-5, atol=1e-7):
            local_scale = np.ones(3, dtype=float)

        scenic_position = np.asarray(scenic_position, dtype=float)

        # If the asset's geometry center is offset from its root prim,
        # scaling changes that offset. We compensate so the final visual bbox
        # center lands at Scenic's obj.position.
        scaled_center_offset_local = native_center * local_scale
        scaled_center_offset_world = self.rotate_vector_by_wxyz_quat(
            orientation,
            scaled_center_offset_local,
        )

        root_position = scenic_position - scaled_center_offset_world

        return root_position, local_scale, native_size, native_center

    def create_generic_object(self, obj):
        raise NotImplementedError

    def create_robot(self, obj):
        raise NotImplementedError

    def create_create3(self, obj):
        raise NotImplementedError

    def create_kaya(self, obj):
        raise NotImplementedError

    def create_franka_panda(self, obj):
        raise NotImplementedError

    def create_ur5e(self, obj):
        raise NotImplementedError

    def create_ground_plane(self, obj):
        raise NotImplementedError

    def apply_robot_control(self, sim, obj, command):
        raise NotImplementedError

    def apply_wheeled_control(self, sim, obj, command):
        raise NotImplementedError

    def move_franka_pick_place(
        self,
        sim,
        obj,
        target_object,
        goal_position,
        end_effector_offset=None,
        end_effector_orientation=None,
    ):
        raise NotImplementedError

    def move_franka_end_effector(self, sim, obj, position, orientation=None):
        raise NotImplementedError

    def set_franka_gripper(self, sim, obj, opened):
        raise NotImplementedError

    def set_franka_arm_joint_positions(self, sim, obj, joint_positions):
        raise NotImplementedError

    def hold_franka_position(self, sim, obj):
        raise NotImplementedError

    def get_franka_end_effector_pose(self, sim, obj):
        raise NotImplementedError

    def get_franka_gripper_positions(self, sim, obj):
        raise NotImplementedError

    def franka_gripper_target_positions(self, opened):
        raise NotImplementedError

    def move_ur5e_end_effector(self, sim, obj, position, orientation=None):
        raise NotImplementedError

    def set_ur5e_gripper(self, sim, obj, opened):
        raise NotImplementedError

    def set_ur5e_arm_joint_positions(self, sim, obj, joint_positions):
        raise NotImplementedError

    def hold_ur5e_position(self, sim, obj):
        raise NotImplementedError

    def get_ur5e_end_effector_pose(self, sim, obj):
        raise NotImplementedError

    def get_ur5e_gripper_positions(self, sim, obj):
        raise NotImplementedError

    def ur5e_gripper_target_positions(self, opened):
        raise NotImplementedError

    def get_physics_properties(self, world, obj):
        raise NotImplementedError

    def articulation_action(self, **kwargs):
        return IsaacArticulationAction(dict(kwargs))
