from __future__ import annotations

import os
import re
from urllib.parse import urlparse

import trimesh

from scenic.core.regions import MeshVolumeRegion
from scenic.core.simulators import SimulationCreationError
from scenic.simulators.isaac.backends.base import IsaacBackend
import scenic.simulators.isaac.utils as scenic_utils


class LabBackend(IsaacBackend):
    
    name = "lab"

    def __init__(self):
        super().__init__()
        self.app_launcher = None

    def ensure_app(
        self,
        *,
        headless=False,
        device=None,
        app_launcher_args=None,
    ):
        """Launch Isaac Sim once through Isaac Lab's AppLauncher."""
        if self._simulation_app is not None:
            return self._simulation_app

        from isaaclab.app import AppLauncher

        launcher_args = {"headless": headless}
        if device is not None:
            launcher_args["device"] = device
        launcher_args.update(app_launcher_args or {})

        self.app_launcher = AppLauncher(launcher_args)
        self._simulation_app = self.app_launcher.app
        return self._simulation_app

    def close_app(self):
        if self._simulation_app is not None:
            self._simulation_app.close()
            self._simulation_app = None
            self.app_launcher = None

    def asset_prim_path(self, asset_name: str, num_envs: int | None) -> str:
        if int(num_envs or 1) == 1:
            return f"/World/envs/env_0/{asset_name}"
        return f"{{ENV_REGEX_NS}}/{asset_name}"

    def resolve_usd_path(self, source) -> str:
        source = os.fspath(source)

        if urlparse(source).scheme:
            return source

        if source.startswith("Isaac/"):
            return self.asset_path(source)

        return os.path.abspath(source)

    def usd_path_for_object(self, obj) -> str | None:
        if getattr(obj, "usd_path", None):
            return self.resolve_usd_path(obj.usd_path)

        if getattr(obj, "isaac_asset_path", None):
            return self.resolve_usd_path(obj.isaac_asset_path)

        return None

    def ensure_environment_mesh_paths(
        self,
        environment_usd_path,
        environment_mesh_path=None,
        environment_info_path=None,
        *,
        headless=True,
        overwrite=False,
    ):
        default_mesh_path, default_info_path = scenic_utils.default_environment_mesh_paths(
            environment_usd_path
        )
        mesh_path = (
            scenic_utils.resolvedPath(environment_mesh_path)
            if environment_mesh_path
            else default_mesh_path
        )
        info_path = (
            scenic_utils.resolvedPath(environment_info_path)
            if environment_info_path
            else default_info_path
        )

        if not overwrite and scenic_utils.environment_outputs_current(
            environment_usd_path, mesh_path, info_path
        ):
            return mesh_path, info_path

        mesh_path.parent.mkdir(parents=True, exist_ok=True)
        info_path.parent.mkdir(parents=True, exist_ok=True)

        self.ensure_app(headless=headless)
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("omni.kit.asset_converter")
        from scenic.simulators.isaac.backends.core_51_usd_to_mesh import (
            convert_environment_usd,
        )

        convert_environment_usd(
            self.kit_usd_path(environment_usd_path),
            str(mesh_path),
            str(info_path),
            overwrite=True,
            backend_name=self.name,
            open_stage_func=self._open_stage_for_conversion,
        )
        return mesh_path, info_path

    def _open_stage_for_conversion(self, usd_path):
        import isaacsim.core.experimental.utils.stage as stage_utils

        result = stage_utils.open_stage(usd_path)
        return result[0] if isinstance(result, tuple) else bool(result)

    def scenic_pose(self, obj):
        pos = getattr(obj, "position", None)
        if pos is None:
            position = (0.0, 0.0, 0.0)
        else:
            position = (
                float(getattr(pos, "x", 0.0)),
                float(getattr(pos, "y", 0.0)),
                float(getattr(pos, "z", 0.0)),
            )

        orientation = self.scenic_to_isaac_orientation(
            obj.orientation,
            initial_rotation=getattr(obj, "initial_rotation", None),
        )
        return position, tuple(float(value) for value in orientation)

    def scenic_color(self, obj):
        color = getattr(obj, "color", None)
        if color is None:
            return None
        r, g, b = color[:3]
        return (float(r), float(g), float(b))

    def preview_surface_cfg(self, obj):
        color = self.scenic_color(obj)
        if color is None:
            return None

        import isaaclab.sim as sim_utils

        return sim_utils.PreviewSurfaceCfg(
            diffuse_color=color,
            roughness=0.5,
            metallic=0.0,
        )

    # ---------------------------------------------------------------------
    # Asset cfg builders
    # ---------------------------------------------------------------------

    def make_environment_cfg(self, environment_usd_path):
        """Create a static environment asset cloned into every Lab environment."""
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg

        usd_path = self.resolve_usd_path(environment_usd_path)
        if not urlparse(usd_path).scheme and not os.path.isfile(usd_path):
            raise SimulationCreationError(
                f"Isaac Lab environment USD does not exist or is not a file: {usd_path!r}"
            )

        return AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/ScenicEnvironment",
            spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        )

    def make_asset_cfg(self, obj, asset_name: str, *, num_envs: int | None, tmp_mesh_dir: str):
        blueprint = obj.blueprint

        if blueprint == "GroundPlane":
            return self.make_ground_plane_cfg(obj, asset_name, num_envs=num_envs)

        if blueprint == "Robot":
            return self.make_robot_cfg(obj, asset_name, num_envs=num_envs)

        return self.make_object_cfg(
            obj,
            asset_name,
            num_envs=num_envs,
            tmp_mesh_dir=tmp_mesh_dir,
        )

    def make_ground_plane_cfg(self, obj, asset_name: str, *, num_envs: int | None):
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg

        pos, rot = self.scenic_pose(obj)

        return AssetBaseCfg(
            prim_path=self.asset_prim_path(asset_name, num_envs),
            spawn=sim_utils.CuboidCfg(
                size=(obj.width, obj.length, obj.height),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=self.preview_surface_cfg(obj),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot),
        )

    def make_object_cfg(self, obj, asset_name: str, *, num_envs: int | None, tmp_mesh_dir: str):
        import isaaclab.sim as sim_utils
        from isaaclab.assets import AssetBaseCfg, RigidObjectCfg

        prim_path = self.asset_prim_path(asset_name, num_envs)
        has_usd_asset = bool(
            getattr(obj, "usd_path", None) or getattr(obj, "isaac_asset_path", None)
        )

        if not has_usd_asset:
            self.convert_mesh_object_to_usd(obj, tmp_mesh_dir)

        usd_path = self.usd_path_for_object(obj)
        if usd_path is None:
            raise SimulationCreationError(
                f"Cannot convert Scenic object {getattr(obj, 'name', obj)!r} "
                "into an Isaac Lab asset cfg."
            )

        if has_usd_asset and getattr(obj, "physics", False):
            usd_path = self.make_rigid_usd_wrapper(
                usd_path,
                obj,
                asset_name,
                tmp_mesh_dir,
            )

        scenic_position, rot = self.scenic_pose(obj)
        pos, scale, _, _ = self.compute_usd_asset_scale_and_root_position(
            obj,
            usd_path,
            scenic_position,
            rot,
        )
        pos = tuple(float(value) for value in pos)
        scale = tuple(float(value) for value in scale)
        spawn = sim_utils.UsdFileCfg(
            usd_path=usd_path,
            scale=scale,
            visual_material=self.preview_surface_cfg(obj),
        )

        if getattr(obj, "physics", False):
            return RigidObjectCfg(
                prim_path=prim_path,
                spawn=spawn,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos, rot=rot),
            )

        return AssetBaseCfg(
            prim_path=prim_path,
            spawn=spawn,
            init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot),
        )

    def make_robot_cfg(self, obj, asset_name: str, *, num_envs: int | None):
        import isaaclab.sim as sim_utils
        from isaaclab.assets import ArticulationCfg
        from isaaclab.actuators import ImplicitActuatorCfg

        usd_path = self.usd_path_for_object(obj)
        if usd_path is None:
            raise SimulationCreationError(
                f"Robot {getattr(obj, 'name', obj)!r} needs usd_path or isaac_asset_path."
            )

        pos, rot = self.scenic_pose(obj)

        return ArticulationCfg(
            prim_path=self.asset_prim_path(asset_name, num_envs),
            spawn=sim_utils.UsdFileCfg(
                usd_path=usd_path,
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                ),
                visual_material=self.preview_surface_cfg(obj),
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=pos, rot=rot),
            actuators={
                "all_joints": ImplicitActuatorCfg(
                    joint_names_expr=[".*"],
                    effort_limit_sim=None,
                    velocity_limit_sim=None,
                    stiffness=0.0,
                    damping=100.0,
                )
            },
        )

    def patch_asset_initial_pose(self, lab_asset_cfg, obj):
        if lab_asset_cfg.init_state is None:
            initial_state_cls = type(lab_asset_cfg).InitialStateCfg
            lab_asset_cfg.init_state = initial_state_cls()

        pos, rot = self.scenic_pose(obj)
        lab_asset_cfg.init_state.pos = pos
        lab_asset_cfg.init_state.rot = rot

    def convert_mesh_object_to_usd(self, obj, tmp_mesh_dir: str):
        os.makedirs(tmp_mesh_dir, exist_ok=True)

        mesh = MeshVolumeRegion(
            mesh=obj.shape.mesh,
            dimensions=(obj.width, obj.length, obj.height),
        ).mesh

        obj_path = os.path.join(tmp_mesh_dir, f"{obj.name}.obj")
        usd_path = os.path.join(tmp_mesh_dir, f"{obj.name}.usd")

        trimesh.exchange.export.export_mesh(mesh, obj_path)

        if not self.convert_sync(obj_path, usd_path, load_materials=True):
            raise SimulationCreationError(
                f"Unable to convert mesh for {obj.name} into a USD asset."
            )

        if getattr(obj, "physics", False):
            self.apply_rigid_body_api_to_usd(usd_path, obj)

        obj.usd_path = usd_path

    def make_rigid_usd_wrapper(self, usd_path, obj, asset_name, tmp_mesh_dir):
        """Create a local rigid-body wrapper without modifying the source USD."""
        from pxr import Usd, UsdGeom

        os.makedirs(tmp_mesh_dir, exist_ok=True)
        wrapper_path = os.path.join(tmp_mesh_dir, f"{asset_name}_rigid.usd")

        stage = Usd.Stage.CreateNew(wrapper_path)
        root_prim = UsdGeom.Xform.Define(stage, "/Asset").GetPrim()
        root_prim.GetReferences().AddReference(usd_path)
        stage.SetDefaultPrim(root_prim)
        stage.GetRootLayer().Save()

        self.apply_rigid_body_api_to_usd(wrapper_path, obj)
        return wrapper_path

    def apply_rigid_body_api_to_usd(self, usd_path, obj):
        """Patch a local USD so Isaac Lab can load it as a RigidObjectCfg.

        RigidObjectCfg requires exactly one UsdPhysics.RigidBodyAPI below its
        configured prim. CollisionAPI is applied to mesh children and MassAPI to
        the rigid body prim.
        """
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.Open(usd_path)
        if stage is None:
            raise SimulationCreationError(f"Could not open USD: {usd_path}")

        root_prim = stage.GetDefaultPrim()

        if not root_prim or not root_prim.IsValid():
            children = list(stage.GetPseudoRoot().GetChildren())
            if not children:
                raise SimulationCreationError(
                    f"USD for {getattr(obj, 'name', obj)!r} has no root prim: {usd_path}"
                )
            root_prim = children[0]
            stage.SetDefaultPrim(root_prim)

        rigid_body_prims = [
            prim
            for prim in Usd.PrimRange(root_prim)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        if len(rigid_body_prims) > 1:
            raise SimulationCreationError(
                f"USD asset for {getattr(obj, 'name', obj)!r} contains multiple rigid bodies "
                "and cannot be loaded as one Isaac Lab RigidObjectCfg."
            )

        if rigid_body_prims:
            rigid_body_prim = rigid_body_prims[0]
        else:
            UsdPhysics.RigidBodyAPI.Apply(root_prim)
            rigid_body_prim = root_prim

        mass_api = UsdPhysics.MassAPI(rigid_body_prim)
        if not mass_api:
            mass_api = UsdPhysics.MassAPI.Apply(rigid_body_prim)

        mass = getattr(obj, "mass", None)
        density = getattr(obj, "density", None)

        if mass is None and density is not None:
            width = float(getattr(obj, "width", 1.0))
            length = float(getattr(obj, "length", 1.0))
            height = float(getattr(obj, "height", 1.0))
            mass = float(density) * width * length * height

        if mass is not None:
            mass_api.CreateMassAttr(float(mass))

        mesh_prims = []
        for prim in Usd.PrimRange(root_prim):
            if prim.IsA(UsdGeom.Mesh):
                mesh_prims.append(prim)

        if not mesh_prims:
            # Fallback: try collision on root, although this only works if root has geometry.
            if not root_prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(root_prim)
        else:
            for mesh_prim in mesh_prims:
                if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(mesh_prim)

                try:
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI(mesh_prim)
                    if not mesh_collision_api:
                        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
                    mesh_collision_api.CreateApproximationAttr("convexHull")
                except Exception:
                    pass

        stage.GetRootLayer().Save()
    
    def apply_robot_control(self, sim, obj, command):
        """Buffer a robot command for application immediately before the Lab step."""
        if getattr(obj, "wheel_controller", None) or callable(
            getattr(obj, "control", None)
        ):
            self.apply_wheeled_control(sim, obj, command)

    def apply_wheeled_control(self, sim, obj, command):
        sim._pending_robot_commands[getattr(obj, "name", id(obj))] = (obj, command)

    def apply_differential_drive_command(self, asset, obj, command, *, debug=False):
        import torch

        throttle, steering = command

        radius = float(getattr(obj, "wheel_radius", 0.03575))
        base = float(getattr(obj, "wheel_base", 0.233))

        left_vel = ((2.0 * float(throttle)) - (float(steering) * base)) / (2.0 * radius)
        right_vel = ((2.0 * float(throttle)) + (float(steering) * base)) / (2.0 * radius)

        wheel_names = (
            list(getattr(obj, "wheel_dof_names", []))
            or list(getattr(obj, "wheel_joint_names", []))
            or ["left_wheel_joint", "right_wheel_joint"]
        )

        joint_ids = self.find_joint_ids(asset, wheel_names)

        if len(joint_ids) < 2:
            if debug:
                print("[SCENIC ISAAC LAB DEBUG] Could not resolve wheel joints for", getattr(obj, "name", obj))
                print("[SCENIC ISAAC LAB DEBUG] requested wheel names:", wheel_names)
                print("[SCENIC ISAAC LAB DEBUG] available joints:", getattr(asset, "joint_names", None))
            return

        target = torch.zeros((int(asset.num_instances), 2), device=asset.device)
        target[:, 0] = left_vel
        target[:, 1] = right_vel

        asset.set_joint_velocity_target(target, joint_ids=joint_ids[:2])
        asset.write_data_to_sim()

    def apply_articulation_action(self, sim, obj, action):
        asset = sim._asset_for_scenic_object(obj)
        self._apply_articulation_action(asset, action)

    def _apply_articulation_action(self, asset, action):
        """Apply a generic articulation action to every environment."""
        import torch

        joint_ids = action.get("joint_indices", action.get("dof_indices"))
        if hasattr(joint_ids, "tolist"):
            joint_ids = joint_ids.tolist()

        applied = False
        for field, index_field, setter_name in (
            ("joint_positions", "joint_position_indices", "set_joint_position_target"),
            ("joint_velocities", "joint_velocity_indices", "set_joint_velocity_target"),
            ("joint_efforts", "joint_effort_indices", "set_joint_effort_target"),
        ):
            values = action.get(field)
            if values is None:
                continue

            field_joint_ids = action.get(index_field, joint_ids)
            if hasattr(field_joint_ids, "tolist"):
                field_joint_ids = field_joint_ids.tolist()

            target = torch.as_tensor(values, dtype=torch.float32, device=asset.device)
            if target.ndim == 1:
                target = target.unsqueeze(0).repeat(int(asset.num_instances), 1)

            getattr(asset, setter_name)(target, joint_ids=field_joint_ids)
            applied = True

        if applied:
            asset.write_data_to_sim()

    def find_joint_ids(self, asset, joint_names):
        joint_ids, _ = asset.find_joints(joint_names, preserve_order=True)
        return joint_ids.tolist() if hasattr(joint_ids, "tolist") else list(joint_ids)

    def _safe_asset_name(self, name: str):
        name = str(name)
        name = re.sub(r"\W+", "_", name)
        if not name:
            name = "scenic_object"
        if name[0].isdigit():
            name = f"obj_{name}"
        return name

    def _tensor_row(self, tensor, env_id: int, default):
        if tensor is None:
            return tuple(default)
        row = tensor[env_id]
        if hasattr(row, "detach"):
            row = row.detach().cpu().tolist()
        return tuple(float(v) for v in row)
