from dataclasses import dataclass, field
import math
import os
import numpy as np

from scenic.simulators.isaac.backends.base import IsaacArticulationAction, IsaacBackend
import scenic.simulators.isaac.utils as scenic_utils


@dataclass
class ExperimentalObjectHandle:
    name: str
    prim_path: str
    wrapper: object
    kind: str
    metadata: dict = field(default_factory=dict)


@dataclass
class ExperimentalWorld:
    app: object
    timestep: float
    objects: dict = field(default_factory=dict)
    simulation_time: float = 0.0

    def get_object(self, name):
        return self.objects[name]


@dataclass
class FrankaPickPlaceState:
    stage: int = 0
    stage_steps: int = 0
    done: bool = False
    end_effector_orientation: object = None
    pick_position: object = None
    place_position: object = None


def _quat_mul(a, b):
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.stack([w, x, y, z], axis=-1)


def _quat_conjugate(q):
    return np.concatenate((q[:, :1], -q[:, 1:]), axis=-1)


def _position_array(position):
    if hasattr(position, "x") and hasattr(position, "y") and hasattr(position, "z"):
        return np.array([position.x, position.y, position.z], dtype=float)
    return np.asarray(position, dtype=float).reshape(-1)[:3]


UR5E_ARM_DOF_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
UR5E_DEFAULT_ARM_POSE = np.array(
    [-np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, np.pi / 2.0, 0.0],
    dtype=float,
)
UR5E_DOWNWARD_ORIENTATION = np.array([0.0, 0.70710678, 0.70710678, 0.0], dtype=float)
UR5E_TCP_OFFSET = np.array([0.0, 0.0, 0.135], dtype=float)
UR5E_GRIPPER_OPEN_POSITION = 0.0
UR5E_GRIPPER_CLOSED_POSITION = np.deg2rad(40.0)
UR5E_GRIPPER_FULLY_CLOSED_POSITION = 47.0
UR5E_GRIPPER_CLOSE_VELOCITY = np.deg2rad(90.0)
UR5E_GRIPPER_OPEN_VELOCITY = -np.deg2rad(45.0)
UR5E_GRIPPER_CONTACT_MATERIAL_PATH = "/World/UR5eRobotiqGripMaterial"
UR5E_OBJECT_CONTACT_MATERIAL_PATH = "/World/UR5ePickObjectGripMaterial"
UR5E_GRIPPER_STATIC_FRICTION = 2.0
UR5E_GRIPPER_DYNAMIC_FRICTION = 2.0
UR5E_OBJECT_STATIC_FRICTION = 2.0
UR5E_OBJECT_DYNAMIC_FRICTION = 2.0
UR5E_CONTACT_OFFSET = 0.002
UR5E_REST_OFFSET = 0.0
UR5E_PICK_OBJECT_MASS_KG = 0.475
UR5E_GRIPPER_MAX_FORCE = 5.0
UR5E_GRIPPER_STIFFNESS = 0.0
UR5E_GRIPPER_DAMPING = 5000.0
UR5E_GRIPPER_MAX_JOINT_VELOCITY_DEG_PER_SEC = 130.0
UR5E_MIMIC_NATURAL_FREQUENCY = 0.0
UR5E_MIMIC_DAMPING_RATIO = 0.0
UR5E_OUTER_FINGER_PARALLEL_STIFFNESS = 0.05
UR5E_GRIPPER_VARIANT = "Robotiq_2f_85"
UR5E_GRIPPER_PRIM = "Gripper/Robotiq_2F_85"
UR5E_CONTROL_PRIM = "wrist_3_link"
UR5E_CONTROL_LINK_NAME = "wrist_3_link"
UR5E_IK_STEP_SCALE = 0.25
UR5E_IK_DAMPING = 0.08


def _differential_inverse_kinematics(
    jacobian_end_effector,
    current_position,
    current_orientation,
    goal_position,
    goal_orientation=None,
    damping=0.05,
    scale=1.0,
):
    goal_orientation = (
        current_orientation if goal_orientation is None else goal_orientation
    )
    q = _quat_mul(goal_orientation, _quat_conjugate(current_orientation))
    error = np.expand_dims(
        np.concatenate(
            [goal_position - current_position, q[:, 1:] * np.sign(q[:, [0]])],
            axis=-1,
        ),
        axis=2,
    )
    transpose = np.swapaxes(jacobian_end_effector, 1, 2)
    lmbda = np.eye(jacobian_end_effector.shape[1]) * (damping**2)
    return (
        scale
        * transpose
        @ np.linalg.inv(jacobian_end_effector @ transpose + lmbda)
        @ error
    ).squeeze(-1)


class Experimental60Backend(IsaacBackend):
    """Isaac Sim 6.0 backend implemented with Core Experimental APIs."""

    name = "experimental_60"

    def create_world(self, timestep):
        self._ensure_stage()
        return ExperimentalWorld(app=self._simulation_app, timestep=timestep)

    def _ensure_stage(self):
        import isaacsim.core.experimental.utils.stage as stage_utils

        if stage_utils.get_current_stage() is None:
            stage_utils.create_new_stage(template="sunlight")

    def open_environment_stage(self, usd_path):
        import isaacsim.core.experimental.utils.stage as stage_utils

        if self._stage_already_open(stage_utils, usd_path):
            return True

        opened, stage = stage_utils.open_stage(usd_path)
        if not opened:
            return False
        stage.SetEditTarget(stage.GetSessionLayer())
        self._environment_usd_path = usd_path
        return True

    def _stage_already_open(self, stage_utils, usd_path):
        if getattr(self, "_environment_usd_path", None) != usd_path:
            return False
        stage = stage_utils.get_current_stage()
        if stage is None:
            return False
        stage.SetEditTarget(stage.GetSessionLayer())
        return True

    def enable_extension(self, name):
        import omni.kit.app

        manager = omni.kit.app.get_app().get_extension_manager()
        if hasattr(manager, "set_extension_enabled_immediate"):
            manager.set_extension_enabled_immediate(name, True)
        else:
            manager.set_extension_enabled(name, True)

    def initialize_physics(self, world, objects):
        self._configure_ur5e_pick_objects_for_world(world, objects)
        if world.app is not None:
            world.app.update()

    def _configure_ur5e_pick_objects_for_world(self, world, objects):
        if not any(handle.kind == "ur5e" for handle in world.objects.values()):
            return

        import isaacsim.core.experimental.utils.stage as stage_utils

        stage = stage_utils.get_current_stage()
        object_by_name = {obj.name: obj for obj in objects}
        pick_object_paths = []
        for name, handle in world.objects.items():
            obj = object_by_name.get(name)
            if handle.kind == "object" and obj is not None and obj.physics:
                pick_object_paths.append(handle.prim_path)
        if pick_object_paths:
            self._configure_ur5e_pick_object_contact(stage, pick_object_paths)

    def play_world(self, world):
        import omni.timeline

        omni.timeline.get_timeline_interface().play()
        if world.app is not None:
            world.app.update()

    def step_world(self, world):
        if world.app is not None:
            world.app.update()
        world.simulation_time += world.timestep

    def stop_and_clear_world(self, world):
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()

        for handle in list(world.objects.values()):
            stage_utils.delete_prim(handle.prim_path)
        world.objects.clear()

    def add_object(self, world, obj):
        if isinstance(obj, ExperimentalObjectHandle):
            world.objects[obj.name] = obj

    def run_coroutine(self, coro):
        if self._simulation_app is not None and hasattr(
            self._simulation_app, "run_coroutine"
        ):
            return self._simulation_app.run_coroutine(coro)

        return super().run_coroutine(coro)

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

        if not self.kit_app_running():
            self.get_simulation_app(headless=headless)

        self.enable_extension("omni.kit.asset_converter")
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

        opened, stage = stage_utils.open_stage(usd_path)
        return opened

    def create_generic_object(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import RigidPrim, XformPrim

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path)

        scenic_position = scenic_utils.vectorToArray(obj.position)
        orientation = self.scenic_to_isaac_orientation(obj.orientation)
        geometry_paths = self._geometry_paths_under(prim_path)
        self._apply_collisions_to_geometry(geometry_paths)

        # Compute scale from Scenic dimensions to native USD dimensions.
        root_position, local_scale, native_size, native_center = (
            self.compute_usd_scale_and_root_position(
                obj,
                prim_path,
                scenic_position,
                orientation,
            )
        )

        if obj.physics:
            wrapper = RigidPrim(
                prim_path,
                positions=root_position,
                orientations=orientation,
                reset_xform_op_properties=True,
            )
            if obj.mass is not None:
                wrapper.set_masses(obj.mass)
            if obj.density is not None:
                wrapper.set_densities(obj.density)
            velocity = scenic_utils.vectorToArray(obj.velocity)
            wrapper.set_velocities(linear_velocities=velocity)
        else:
            wrapper = XformPrim(
                prim_path,
                positions=root_position,
                orientations=orientation,
                reset_xform_op_properties=True,
            )
            self.disable_rigid_body(prim_path)
        
        wrapper.set_world_poses(positions=root_position, orientations=orientation)
        wrapper.set_local_scales(local_scale)

        if obj.color:
            self.apply_visual_material(wrapper, obj, geometry_paths=geometry_paths)
        return ExperimentalObjectHandle(obj.name, prim_path, wrapper, "object")

    def _geometry_paths_under(self, prim_path):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from pxr import Usd, UsdGeom

        stage = stage_utils.get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        paths = []
        for descendant in Usd.PrimRange(prim):
            if descendant.IsA(UsdGeom.Gprim):
                paths.append(str(descendant.GetPath()))
        return paths

    def _apply_collisions_to_geometry(self, geometry_paths):
        if not geometry_paths:
            return
        from isaacsim.core.experimental.prims import GeomPrim

        geom = GeomPrim(geometry_paths, apply_collision_apis=True)
        geom.set_collision_approximations(["convexDecomposition"])

    def disable_rigid_body(self, prim_path):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from pxr import Usd, UsdPhysics

        prim = stage_utils.get_current_stage().GetPrimAtPath(prim_path)
        for descendant in Usd.PrimRange(prim):
            if descendant.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_api = UsdPhysics.RigidBodyAPI(descendant)
                rigid_body_api.CreateRigidBodyEnabledAttr(False)

    def apply_visual_material(self, wrapper, obj, geometry_paths=None):
        from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
        from isaacsim.core.experimental.prims import GeomPrim

        material = PreviewSurfaceMaterial(f"/World/material/{obj.name}")
        color = scenic_utils.colorToArray(obj.color)
        material.set_input_values("diffuseColor", color[:3])
        if len(color) > 3:
            material.set_input_values("opacity", [color[3]])

        if geometry_paths:
            GeomPrim(geometry_paths).apply_visual_materials(material)
        else:
            wrapper.apply_visual_materials(material)

    def create_robot(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path)
        wrapper = Articulation(
            prim_path,
            positions=scenic_utils.vectorToArray(obj.position),
            orientations=self.scenic_to_isaac_orientation(obj.orientation),
            reset_xform_op_properties=True,
        )
        if obj.control:
            obj.controller = obj.control
        return ExperimentalObjectHandle(obj.name, prim_path, wrapper, "articulation")

    def create_create3(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation

        prim_path = f"/World/{obj.name}"
        stage_utils.add_reference_to_stage(
            usd_path=self.asset_path("Isaac/Robots/iRobot/Create3/create_3.usd"),
            path=prim_path,
        )
        wrapper = Articulation(
            prim_path,
            positions=scenic_utils.vectorToArray(obj.position),
            orientations=self.scenic_to_isaac_orientation(obj.orientation),
            reset_xform_op_properties=True,
        )
        metadata = {
            "controller": "differential",
            "wheel_radius": 0.03575,
            "wheel_base": 0.233,
            "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
        }
        if obj.color:
            self.apply_visual_material(
                wrapper, obj, geometry_paths=self._geometry_paths_under(prim_path)
            )
        return ExperimentalObjectHandle(obj.name, prim_path, wrapper, "articulation", metadata)

    def create_kaya(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation

        prim_path = f"/World/{obj.name}"
        stage_utils.add_reference_to_stage(
            usd_path=self.asset_path("Isaac/Robots/NVIDIA/Kaya/kaya.usd"),
            path=prim_path,
        )
        wrapper = Articulation(
            prim_path,
            positions=scenic_utils.vectorToArray(obj.position),
            orientations=self.scenic_to_isaac_orientation(
                obj.orientation, initial_rotation=[np.pi / 2, 0, 0]
            ),
            reset_xform_op_properties=True,
        )
        metadata = {
            "controller": "holonomic",
            "wheel_radius": 0.04,
            "base_radius": 0.125,
            "wheel_dof_names": ["axle_0_joint", "axle_1_joint", "axle_2_joint"],
            "wheel_angles": [0.0, 2.0 * np.pi / 3.0, 4.0 * np.pi / 3.0],
        }
        if obj.color:
            self.apply_visual_material(
                wrapper, obj, geometry_paths=self._geometry_paths_under(prim_path)
            )
        return ExperimentalObjectHandle(obj.name, prim_path, wrapper, "articulation", metadata)

    def create_franka_panda(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation, RigidPrim

        prim_path = f"/World/{obj.name}"
        root_position = self._franka_root_position(obj)
        root_orientation = self.scenic_to_isaac_orientation(obj.orientation)

        stage_utils.add_reference_to_stage(
            usd_path=self.asset_path(
                "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
            ),
            path=prim_path,
            variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
        )
        wrapper = Articulation(
            prim_path,
            positions=root_position,
            orientations=root_orientation,
            reset_xform_op_properties=True,
        )
        default_dof_positions = np.array(
            [0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.05, 0.05],
            dtype=float,
        )
        wrapper.set_default_state(dof_positions=default_dof_positions)

        end_effector = RigidPrim(f"{prim_path}/panda_hand")
        end_effector_link_index = wrapper.get_link_indices("panda_hand").list()[0]
        metadata = {
            "end_effector": end_effector,
            "end_effector_link_index": end_effector_link_index,
            "arm_dof_indices": list(range(7)),
            "arm_dof_count": 7,
            "gripper_dof_indices": self._dof_indices(
                wrapper, ["panda_finger_joint1", "panda_finger_joint2"]
            ),
            "default_dof_positions": default_dof_positions,
            "open_gripper_positions": np.array([0.05, 0.05], dtype=float),
            "closed_gripper_positions": np.array([0.01, 0.01], dtype=float),
            "downward_orientation": np.array([0.0, 1.0, 0.0, 0.0], dtype=float),
            "tcp_offset": np.array([0.0, 0.0, 0.0877], dtype=float),
        }
        return ExperimentalObjectHandle(
            obj.name,
            prim_path,
            wrapper,
            "franka",
            metadata,
        )

    def _franka_root_position(self, obj):
        position = scenic_utils.vectorToArray(obj.position)
        position[2] -= obj.height / 2
        return position

    def create_ur5e(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation, RigidPrim

        prim_path = f"/World/{obj.name}"
        base_position = self._ur5e_base_position(obj)
        robot_position = base_position
        root_orientation = self.scenic_to_isaac_orientation(obj.orientation)

        robot_prim = stage_utils.add_reference_to_stage(
            usd_path=self.asset_path("Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"),
            path=prim_path,
        )
        self._set_required_variant(robot_prim, "Gripper", UR5E_GRIPPER_VARIANT)

        stage = stage_utils.get_current_stage()
        self._require_stage_prim(stage, f"{prim_path}/{UR5E_CONTROL_PRIM}")
        self._configure_ur5e_gripper_attachment(stage, prim_path)
        self._configure_ur5e_default_joint_pose(stage, prim_path)
        self._configure_ur5e_closed_loop_gripper(stage, prim_path)
        self._configure_ur5e_gripper_drive(stage, prim_path)
        self._configure_ur5e_gripper_contact(stage, prim_path)

        wrapper = Articulation(
            prim_path,
            positions=robot_position,
            orientations=root_orientation,
            reset_xform_op_properties=True,
        )
        arm_dof_indices = self._dof_indices(wrapper, UR5E_ARM_DOF_NAMES)
        gripper_dof_indices = self._dof_indices(wrapper, ["finger_joint"])
        default_dof_positions = np.zeros(len(wrapper.dof_names), dtype=float)
        for value, dof_index in zip(UR5E_DEFAULT_ARM_POSE, arm_dof_indices):
            default_dof_positions[dof_index] = value
        for dof_index in gripper_dof_indices:
            default_dof_positions[dof_index] = UR5E_GRIPPER_OPEN_POSITION
        wrapper.set_default_state(dof_positions=default_dof_positions)

        end_effector = RigidPrim(f"{prim_path}/{UR5E_CONTROL_PRIM}")
        end_effector_link_index = self._link_index(wrapper, UR5E_CONTROL_LINK_NAME)
        metadata = {
            "end_effector": end_effector,
            "end_effector_link_index": end_effector_link_index,
            "arm_dof_indices": arm_dof_indices,
            "gripper_dof_indices": gripper_dof_indices,
            "default_dof_positions": default_dof_positions,
            "open_gripper_positions": np.array([UR5E_GRIPPER_OPEN_POSITION], dtype=float),
            "closed_gripper_positions": np.array([UR5E_GRIPPER_CLOSED_POSITION], dtype=float),
            "open_gripper_velocity": UR5E_GRIPPER_OPEN_VELOCITY,
            "closed_gripper_velocity": UR5E_GRIPPER_CLOSE_VELOCITY,
            "downward_orientation": UR5E_DOWNWARD_ORIENTATION.copy(),
            "tcp_offset": UR5E_TCP_OFFSET.copy(),
        }
        return ExperimentalObjectHandle(
            obj.name,
            prim_path,
            wrapper,
            "ur5e",
            metadata,
        )

    def _set_required_variant(self, prim, variant_name, selection):
        variant_set = prim.GetVariantSet(variant_name)
        if not variant_set or not variant_set.IsValid():
            raise RuntimeError(f"{prim.GetPath()} has no {variant_name!r} variant set")
        available = list(variant_set.GetVariantNames())
        if selection not in available:
            raise RuntimeError(
                f"{prim.GetPath()} {variant_name!r} variant {selection!r} is missing"
            )
        variant_set.SetVariantSelection(selection)

    def _require_stage_prim(self, stage, prim_path):
        if stage is None:
            raise RuntimeError("Required Isaac USD stage is missing")
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Required Isaac prim is missing: {prim_path}")
        return prim

    def _ur5e_base_position(self, obj):
        position = scenic_utils.vectorToArray(obj.position)
        position[2] -= obj.height / 2
        return position

    def _configure_ur5e_gripper_attachment(self, stage, prim_path):
        from pxr import Gf, Sdf

        joint = self._require_stage_prim(
            stage, f"{prim_path}/joints/robot_gripper_joint"
        )

        def set_quat_attr(attr_name, values):
            attr = joint.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Quatf)
            attr.Set(
                Gf.Quatf(
                    float(values[0]),
                    Gf.Vec3f(float(values[1]), float(values[2]), float(values[3])),
                )
            )

        set_quat_attr("physics:localRot0", (0.70710677, 0.0, 0.0, 0.70710677))
        set_quat_attr("physics:localRot1", (1.0, 0.0, 0.0, 0.0))

    def _configure_ur5e_default_joint_pose(self, stage, prim_path):
        from pxr import Sdf

        for joint_name, angle_deg in zip(
            UR5E_ARM_DOF_NAMES, np.rad2deg(UR5E_DEFAULT_ARM_POSE)
        ):
            joint = self._require_stage_prim(stage, f"{prim_path}/joints/{joint_name}")
            for attr_name in (
                "drive:angular:physics:targetPosition",
                "state:angular:physics:position",
            ):
                attr = joint.GetAttribute(attr_name)
                if not attr or not attr.IsValid():
                    attr = joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
                attr.Set(float(angle_deg))

        gripper_joint = self._require_stage_prim(
            stage, f"{prim_path}/{UR5E_GRIPPER_PRIM}/Joints/finger_joint"
        )
        for attr_name in (
            "drive:angular:physics:targetPosition",
            "state:angular:physics:position",
        ):
            attr = gripper_joint.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = gripper_joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
            attr.Set(float(UR5E_GRIPPER_OPEN_POSITION))

    def _configure_ur5e_gripper_drive(self, stage, prim_path):
        from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

        gripper_root = self._require_stage_prim(stage, f"{prim_path}/{UR5E_GRIPPER_PRIM}")
        found_finger_joint = False

        def set_attr(prim, attr_name, value, value_type):
            attr = prim.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute(attr_name, value_type)
            attr.Set(float(value))
            return attr

        def set_drive_attrs(
            prim,
            max_force,
            stiffness,
            damping,
            target_velocity=0.0,
        ):
            if "PhysicsDriveAPI:angular" not in list(prim.GetAppliedSchemas()):
                UsdPhysics.DriveAPI.Apply(prim, "angular")
            for attr_name, value in (
                ("drive:angular:physics:maxForce", max_force),
                ("drive:angular:physics:stiffness", stiffness),
                ("drive:angular:physics:damping", damping),
                ("drive:angular:physics:targetVelocity", target_velocity),
            ):
                attr = prim.GetAttribute(attr_name)
                if not attr or not attr.IsValid():
                    attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
                attr.Set(float(value))
            drive_type = prim.GetAttribute("drive:angular:physics:type")
            if not drive_type or not drive_type.IsValid():
                drive_type = prim.CreateAttribute(
                    "drive:angular:physics:type", Sdf.ValueTypeNames.Token
                )
            drive_type.Set("force")
            joint_api = (
                PhysxSchema.PhysxJointAPI(prim)
                if prim.HasAPI(PhysxSchema.PhysxJointAPI)
                else PhysxSchema.PhysxJointAPI.Apply(prim)
            )
            joint_api.CreateMaxJointVelocityAttr().Set(
                float(UR5E_GRIPPER_MAX_JOINT_VELOCITY_DEG_PER_SEC)
            )

        for prim in Usd.PrimRange(gripper_root):
            name = prim.GetName()
            if "Joint" not in str(prim.GetTypeName()) and not name.endswith("_joint"):
                continue
            for schema in list(prim.GetAppliedSchemas()):
                if not schema.startswith("PhysxMimicJointAPI:"):
                    continue
                axis = schema.split(":", 1)[1]
                set_attr(
                    prim,
                    f"physxMimicJoint:{axis}:naturalFrequency",
                    UR5E_MIMIC_NATURAL_FREQUENCY,
                    Sdf.ValueTypeNames.Float,
                )
                set_attr(
                    prim,
                    f"physxMimicJoint:{axis}:dampingRatio",
                    UR5E_MIMIC_DAMPING_RATIO,
                    Sdf.ValueTypeNames.Float,
                )
            if name == "finger_joint":
                found_finger_joint = True
                set_drive_attrs(
                    prim,
                    UR5E_GRIPPER_MAX_FORCE,
                    UR5E_GRIPPER_STIFFNESS,
                    UR5E_GRIPPER_DAMPING,
                )
                set_attr(prim, "physics:lowerLimit", 0.0, Sdf.ValueTypeNames.Float)
                set_attr(
                    prim,
                    "physics:upperLimit",
                    UR5E_GRIPPER_FULLY_CLOSED_POSITION,
                    Sdf.ValueTypeNames.Float,
                )
            elif name in ("left_outer_finger_joint", "right_outer_finger_joint"):
                set_drive_attrs(
                    prim,
                    UR5E_GRIPPER_MAX_FORCE,
                    UR5E_OUTER_FINGER_PARALLEL_STIFFNESS,
                    UR5E_GRIPPER_DAMPING,
                )
            elif "finger" in name or "knuckle" in name:
                for attr_name in (
                    "drive:angular:physics:maxForce",
                    "drive:angular:physics:stiffness",
                    "drive:angular:physics:damping",
                    "drive:angular:physics:targetVelocity",
                ):
                    attr = prim.GetAttribute(attr_name)
                    if attr and attr.IsValid():
                        attr.Set(0.0)
        if not found_finger_joint:
            raise RuntimeError(f"Missing Robotiq finger_joint under {gripper_root.GetPath()}")

    def _configure_ur5e_gripper_contact(self, stage, prim_path):
        from omni.physx.scripts import physicsUtils
        from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

        root = self._require_stage_prim(stage, f"{prim_path}/{UR5E_GRIPPER_PRIM}")
        material = UsdShade.Material.Define(stage, UR5E_GRIPPER_CONTACT_MATERIAL_PATH)
        material_prim = material.GetPrim()
        material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
        material_api.CreateStaticFrictionAttr().Set(float(UR5E_GRIPPER_STATIC_FRICTION))
        material_api.CreateDynamicFrictionAttr().Set(float(UR5E_GRIPPER_DYNAMIC_FRICTION))
        material_api.CreateRestitutionAttr().Set(0.0)

        physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
        physx_material_api.CreateFrictionCombineModeAttr().Set("max")
        physx_material_api.CreateRestitutionCombineModeAttr().Set("min")

        for prim in Usd.PrimRange(root):
            path = str(prim.GetPath())
            if prim.IsInstance() and (path.endswith("/visuals") or "/visuals/" in path):
                prim.SetInstanceable(False)

        bound = []
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not any("Collision" in schema for schema in prim.GetAppliedSchemas()):
                continue
            if prim.IsInstanceProxy():
                raise RuntimeError(f"Could not bind UR5e contact material to instance proxy: {prim.GetPath()}")
            physicsUtils.add_physics_material_to_prim(stage, prim, UR5E_GRIPPER_CONTACT_MATERIAL_PATH)
            collision_api = (
                UsdPhysics.CollisionAPI(prim)
                if prim.HasAPI(UsdPhysics.CollisionAPI)
                else UsdPhysics.CollisionAPI.Apply(prim)
            )
            collision_api.CreateCollisionEnabledAttr().Set(True)
            physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_collision_api.CreateContactOffsetAttr().Set(float(UR5E_CONTACT_OFFSET))
            physx_collision_api.CreateRestOffsetAttr().Set(float(UR5E_REST_OFFSET))
            bound.append(str(prim.GetPath()))
        if not bound:
            raise RuntimeError(f"No collision geometry found for UR5e Robotiq gripper under {root.GetPath()}")

    def _configure_ur5e_pick_object_contact(self, stage, prim_paths):
        from omni.physx.scripts import physicsUtils
        from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

        material = UsdShade.Material.Define(stage, UR5E_OBJECT_CONTACT_MATERIAL_PATH)
        material_prim = material.GetPrim()
        material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
        material_api.CreateStaticFrictionAttr().Set(float(UR5E_OBJECT_STATIC_FRICTION))
        material_api.CreateDynamicFrictionAttr().Set(float(UR5E_OBJECT_DYNAMIC_FRICTION))
        material_api.CreateRestitutionAttr().Set(0.0)

        physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
        physx_material_api.CreateFrictionCombineModeAttr().Set("max")
        physx_material_api.CreateRestitutionCombineModeAttr().Set("min")

        for prim_path in prim_paths:
            root = self._require_stage_prim(stage, prim_path)
            rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
            rigid_api.CreateEnableCCDAttr().Set(True)
            rigid_api.CreateSleepThresholdAttr().Set(0.0)

            mass_api = (
                UsdPhysics.MassAPI(root)
                if root.HasAPI(UsdPhysics.MassAPI)
                else UsdPhysics.MassAPI.Apply(root)
            )
            mass_api.CreateMassAttr().Set(float(UR5E_PICK_OBJECT_MASS_KG))

            collision_prims = [
                prim
                for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                if any("Collision" in schema for schema in prim.GetAppliedSchemas())
            ]
            if not collision_prims:
                raise RuntimeError(f"No collision geometry found for UR5e pick object: {prim_path}")
            for prim in collision_prims:
                if prim.IsInstanceProxy():
                    raise RuntimeError(f"Could not bind UR5e pick object material to instance proxy: {prim.GetPath()}")
                physicsUtils.add_physics_material_to_prim(stage, prim, UR5E_OBJECT_CONTACT_MATERIAL_PATH)
                collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                collision_api.CreateContactOffsetAttr().Set(float(UR5E_CONTACT_OFFSET))
                collision_api.CreateRestOffsetAttr().Set(float(UR5E_REST_OFFSET))

    def _configure_ur5e_closed_loop_gripper(self, stage, prim_path):
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics

        base_path = f"{prim_path}/{UR5E_GRIPPER_PRIM}/base_link"
        joint_root_path = f"{prim_path}/{UR5E_GRIPPER_PRIM}/Joints"
        self._require_stage_prim(stage, base_path)
        joint_root = self._require_stage_prim(stage, joint_root_path)
        for body_path in (
            f"{prim_path}/{UR5E_GRIPPER_PRIM}/left_inner_knuckle",
            f"{prim_path}/{UR5E_GRIPPER_PRIM}/right_inner_knuckle",
        ):
            self._require_stage_prim(stage, body_path)

        def set_attr(prim, attr_name, value, value_type):
            attr = prim.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute(attr_name, value_type)
            attr.Set(value)
            return attr

        def configure_joint_common(prim, exclude_from_articulation):
            set_attr(
                prim,
                "physics:excludeFromArticulation",
                bool(exclude_from_articulation),
                Sdf.ValueTypeNames.Bool,
            )
            set_attr(prim, "physics:jointEnabled", True, Sdf.ValueTypeNames.Bool)
            if not prim.HasAPI(PhysxSchema.PhysxJointAPI):
                PhysxSchema.PhysxJointAPI.Apply(prim)

        passive_joint_specs = (
            (
                "left_inner_knuckle_joint",
                f"{prim_path}/{UR5E_GRIPPER_PRIM}/left_inner_knuckle",
                (0.0, -0.0127, 0.06142),
                (0.5, 0.5, -0.5, -0.5),
            ),
            (
                "right_inner_knuckle_joint",
                f"{prim_path}/{UR5E_GRIPPER_PRIM}/right_inner_knuckle",
                (0.0, 0.0127, 0.06142),
                (0.5, -0.5, 0.5, -0.5),
            ),
        )
        for name, body_path, local_pos, local_rot in passive_joint_specs:
            joint_path = f"{joint_root_path}/{name}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if not joint_prim.IsValid():
                joint_prim = UsdPhysics.RevoluteJoint.Define(stage, joint_path).GetPrim()
            joint = UsdPhysics.RevoluteJoint(joint_prim)
            joint.GetBody0Rel().SetTargets([Sdf.Path(base_path)])
            joint.GetBody1Rel().SetTargets([Sdf.Path(body_path)])
            joint.CreateAxisAttr().Set(UsdPhysics.Tokens.z)
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos))
            joint.CreateLocalRot0Attr().Set(
                Gf.Quatf(local_rot[0], Gf.Vec3f(*local_rot[1:]))
            )
            joint.CreateLocalRot1Attr().Set(
                Gf.Quatf(local_rot[0], Gf.Vec3f(*local_rot[1:]))
            )
            configure_joint_common(joint_prim, exclude_from_articulation=True)

        for prim in Usd.PrimRange(joint_root):
            if prim.GetName() in (
                "left_inner_finger_knuckle_joint",
                "right_inner_finger_knuckle_joint",
            ):
                configure_joint_common(prim, exclude_from_articulation=False)

    def _link_index(self, articulation, name):
        indices = articulation.get_link_indices(name).list()
        if len(indices) != 1:
            raise RuntimeError(f"Expected one link named {name!r}, found {len(indices)}")
        return indices[0]

    def create_ground_plane(self, obj):
        from isaacsim.core.experimental.objects import GroundPlane

        wrapper = GroundPlane(
            "/World/GroundPlane",
            sizes=max(obj.width, obj.length),
            positions=[0, 0, 0],
        )
        if obj.color:
            self.apply_visual_material(wrapper, obj)
        return ExperimentalObjectHandle(obj.name, "/World/GroundPlane", wrapper, "ground")

    def apply_robot_control(self, sim, obj, command):
        handle = sim.world.get_object(obj.name)
        if obj.controller is None:
            return
        action = obj.controller(command)
        self._apply_articulation_action(handle.wrapper, action)

    def apply_wheeled_control(self, sim, obj, command):
        handle = sim.world.get_object(obj.name)
        metadata = handle.metadata
        if metadata.get("controller") == "differential":
            throttle, steering = command
            radius = metadata["wheel_radius"]
            base = metadata["wheel_base"]
            velocities = [
                ((2.0 * throttle) - (steering * base)) / (2.0 * radius),
                ((2.0 * throttle) + (steering * base)) / (2.0 * radius),
            ]
        elif metadata.get("controller") == "holonomic":
            forward_speed, lateral_speed, yaw_speed = command
            radius = metadata["wheel_radius"]
            base_radius = metadata["base_radius"]
            velocities = []
            for angle in metadata["wheel_angles"]:
                velocities.append(
                    (
                        math.sin(angle) * forward_speed
                        + math.cos(angle) * lateral_speed
                        + base_radius * yaw_speed
                    )
                    / radius
                )
        else:
            raise RuntimeError(f"{obj.name} has no wheeled controller metadata")

        dof_indices = self._dof_indices(handle.wrapper, metadata["wheel_dof_names"])
        handle.wrapper.set_dof_velocity_targets(velocities, dof_indices=dof_indices)

    def move_franka_pick_place(
        self,
        sim,
        obj,
        target_object,
        goal_position,
        end_effector_offset=None,
        end_effector_orientation=None,
    ):
        handle = sim.world.get_object(obj.name)

        if obj.controller is None:
            obj.controller = FrankaPickPlaceState()
            self._reset_franka(handle)
        if obj.controller.done:
            return

        if end_effector_orientation is None:
            end_effector_orientation = obj.end_effector_orientation
        if end_effector_offset is None:
            end_effector_offset = obj.end_effector_offset

        end_effector_offset = np.array(end_effector_offset, dtype=float)

        self._move_franka_pick_place_helper(
            handle,
            obj.controller,
            sim,
            target_object,
            goal_position,
            end_effector_offset,
            end_effector_orientation,
        )

    def _move_franka_pick_place_helper(
        self,
        handle,
        state,
        sim,
        target_object,
        goal_position,
        end_effector_offset,
        end_effector_orientation=None,
    ):
        if state.pick_position is None:
            target_handle = sim.world.get_object(target_object.name)
            state.pick_position = target_handle.wrapper.get_world_poses()[0].numpy()[0].copy()
        if state.place_position is None:
            state.place_position = scenic_utils.vectorToArray(goal_position).copy()

        cube_position = state.pick_position
        place = state.place_position
        current_position, current_orientation = self._franka_end_effector_pose(handle)

        if end_effector_orientation is not None:
            state.end_effector_orientation = np.array(end_effector_orientation, dtype=float).copy()
        elif state.end_effector_orientation is None:
            state.end_effector_orientation = handle.metadata["downward_orientation"].copy()
        orientation = state.end_effector_orientation

        phases = [
            (cube_position + np.array([0.0, 0.0, 0.2]), "open", 120),
            (cube_position + np.array([0.0, 0.0, 0.1]), "open", 80),
            (None, "closed", 50),
            (cube_position + np.array([0.0, 0.0, 0.5]), "closed", 150),
            (place + np.array([0.0, 0.0, 0.5]), "closed", 180),
            (place + np.array([0.0, 0.0, 0.2]), "closed", 90),
            (None, "open", 20),
        ]

        target, gripper, steps = phases[state.stage]
        if target is not None:
            self._move_franka_end_effector(
                handle,
                current_position,
                current_orientation,
                np.array([target + end_effector_offset], dtype=float),
                np.array([orientation], dtype=float),
            )

        if gripper == "open":
            self._set_franka_gripper(handle, "open")
        else:
            self._set_franka_gripper(handle, "closed")

        state.stage_steps += 1
        if state.stage_steps > steps:
            target, gripper, steps = phases[state.stage]
            print(
                f"Franka stage={state.stage}, steps={state.stage_steps}, "
                f"target={target}, gripper={gripper}"
            )
            state.stage += 1
            state.stage_steps = 0
            if state.stage >= len(phases):
                state.done = True

    def _reset_franka(self, handle):
        handle.wrapper.reset_to_default_state()
        handle.wrapper.set_dof_position_targets(
            handle.metadata["default_dof_positions"]
        )

    def _ensure_franka_primitive_control_ready(self, handle):
        if not handle.metadata.get("primitive_control_ready", False):
            self._reset_franka(handle)
            handle.metadata["primitive_control_ready"] = True

    def _franka_end_effector_pose(self, handle):
        position, orientation = handle.metadata["end_effector"].get_world_poses()
        return position.numpy(), orientation.numpy()

    def _franka_tcp_position(self, handle, control_position, orientation):
        tcp_offset = handle.metadata["tcp_offset"]
        control_position = np.asarray(control_position, dtype=float).reshape(-1)[:3]
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return control_position + self.rotate_vector_by_wxyz_quat(orientation, tcp_offset)

    def _franka_control_position(self, handle, tcp_position, orientation):
        tcp_offset = handle.metadata["tcp_offset"]
        tcp_position = _position_array(tcp_position)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return tcp_position - self.rotate_vector_by_wxyz_quat(orientation, tcp_offset)

    def _move_franka_end_effector(
        self,
        handle,
        current_position,
        current_orientation,
        goal_position,
        goal_orientation=None,
    ):
        franka = handle.wrapper
        metadata = handle.metadata
        current_dof_positions = franka.get_dof_positions().numpy()
        jacobian_matrices = franka.get_jacobian_matrices().numpy()
        jacobian_end_effector = jacobian_matrices[
            :,
            metadata["end_effector_link_index"] - 1,
            :,
            : metadata["arm_dof_count"],
        ]
        delta_dof_positions = _differential_inverse_kinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=current_position,
            current_orientation=current_orientation,
            goal_position=np.asarray(goal_position, dtype=float).reshape(1, -1),
            goal_orientation=(
                None
                if goal_orientation is None
                else np.asarray(goal_orientation, dtype=float).reshape(1, -1)
            ),
        )
        dof_position_targets = (
            current_dof_positions[:, metadata["arm_dof_indices"]]
            + delta_dof_positions
        )
        franka.set_dof_position_targets(
            dof_position_targets,
            dof_indices=metadata["arm_dof_indices"],
        )

    def _set_franka_gripper(self, handle, state):
        positions = handle.metadata[f"{state}_gripper_positions"]
        handle.wrapper.set_dof_position_targets(
            positions,
            dof_indices=handle.metadata["gripper_dof_indices"],
        )

    def move_franka_end_effector(self, sim, obj, position, orientation=None):
        handle = sim.world.get_object(obj.name)
        self._ensure_franka_primitive_control_ready(handle)
        current_position, current_orientation = self._franka_end_effector_pose(handle)
        if orientation is None:
            orientation = handle.metadata["downward_orientation"]
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        goal_position = self._franka_control_position(handle, position, orientation)
        self._move_franka_end_effector(
            handle,
            current_position,
            current_orientation,
            np.array([goal_position], dtype=float),
            np.array([orientation], dtype=float),
        )

    def set_franka_gripper(self, sim, obj, opened):
        handle = sim.world.get_object(obj.name)
        self._ensure_franka_primitive_control_ready(handle)
        self._set_franka_gripper(handle, "open" if opened else "closed")

    def set_franka_arm_joint_positions(self, sim, obj, joint_positions):
        handle = sim.world.get_object(obj.name)
        self._ensure_franka_primitive_control_ready(handle)
        joint_positions = np.asarray(joint_positions, dtype=float).reshape(-1)
        arm_dof_indices = handle.metadata["arm_dof_indices"][: len(joint_positions)]
        handle.wrapper.set_dof_position_targets(
            joint_positions.tolist(),
            dof_indices=arm_dof_indices,
        )

    def hold_franka_position(self, sim, obj):
        handle = sim.world.get_object(obj.name)
        self._ensure_franka_primitive_control_ready(handle)
        arm_dof_indices = handle.metadata["arm_dof_indices"]
        current_dof_positions = handle.wrapper.get_dof_positions().numpy()
        arm_targets = current_dof_positions[:, arm_dof_indices].reshape(-1).tolist()
        handle.wrapper.set_dof_position_targets(
            arm_targets,
            dof_indices=arm_dof_indices,
        )

    def get_franka_end_effector_pose(self, sim, obj):
        handle = sim.world.get_object(obj.name)
        position, orientation = self._franka_end_effector_pose(handle)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return (
            self._franka_tcp_position(handle, position, orientation),
            orientation,
        )

    def get_franka_gripper_positions(self, sim, obj):
        handle = sim.world.get_object(obj.name)
        gripper_dof_indices = handle.metadata["gripper_dof_indices"]
        dof_positions = handle.wrapper.get_dof_positions().numpy()
        return dof_positions[:, gripper_dof_indices].reshape(-1)

    def franka_gripper_target_positions(self, opened):
        if opened:
            return np.array([0.05, 0.05], dtype=float)
        return np.array([0.01, 0.01], dtype=float)

    def _ensure_ur5e_primitive_control_ready(self, handle):
        if not handle.metadata.get("primitive_control_ready", False):
            handle.wrapper.reset_to_default_state()
            handle.wrapper.set_dof_position_targets(
                handle.metadata["default_dof_positions"]
            )
            handle.metadata["primitive_control_ready"] = True

    def _ur5e_end_effector_pose(self, handle):
        position, orientation = handle.metadata["end_effector"].get_world_poses()
        return position.numpy(), orientation.numpy()

    def _tcp_position(self, handle, control_position, orientation):
        tcp_offset = handle.metadata["tcp_offset"]
        control_position = np.asarray(control_position, dtype=float).reshape(-1)[:3]
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return control_position + self.rotate_vector_by_wxyz_quat(orientation, tcp_offset)

    def _control_position(self, handle, tcp_position, orientation):
        tcp_offset = handle.metadata["tcp_offset"]
        tcp_position = _position_array(tcp_position)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return tcp_position - self.rotate_vector_by_wxyz_quat(orientation, tcp_offset)

    def _move_ur5e_end_effector(
        self,
        handle,
        current_position,
        current_orientation,
        goal_position,
        goal_orientation=None,
    ):
        ur5e = handle.wrapper
        metadata = handle.metadata
        current_dof_positions = ur5e.get_dof_positions().numpy()
        jacobian_matrices = ur5e.get_jacobian_matrices().numpy()
        jacobian_link_index = metadata["end_effector_link_index"] - 1
        if jacobian_link_index < 0:
            raise RuntimeError("UR5e control link index cannot be used for Jacobian")
        jacobian_end_effector = np.take(
            jacobian_matrices[:, jacobian_link_index, :, :],
            metadata["arm_dof_indices"],
            axis=-1,
        )
        delta_dof_positions = _differential_inverse_kinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=current_position,
            current_orientation=current_orientation,
            goal_position=np.asarray(goal_position, dtype=float).reshape(1, -1),
            goal_orientation=(
                None
                if goal_orientation is None
                else np.asarray(goal_orientation, dtype=float).reshape(1, -1)
            ),
            damping=UR5E_IK_DAMPING,
            scale=UR5E_IK_STEP_SCALE,
        )
        dof_position_targets = (
            current_dof_positions[:, metadata["arm_dof_indices"]]
            + delta_dof_positions
        )
        ur5e.set_dof_position_targets(
            dof_position_targets,
            dof_indices=metadata["arm_dof_indices"],
        )

    def move_ur5e_end_effector(self, sim, obj, position, orientation=None):
        handle = sim.world.get_object(obj.name)
        self._ensure_ur5e_primitive_control_ready(handle)
        current_position, current_orientation = self._ur5e_end_effector_pose(handle)
        if orientation is None:
            orientation = handle.metadata["downward_orientation"]
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        goal_position = self._control_position(handle, position, orientation)
        self._move_ur5e_end_effector(
            handle,
            current_position,
            current_orientation,
            np.array([goal_position], dtype=float),
            np.array([orientation], dtype=float),
        )

    def set_ur5e_gripper(self, sim, obj, opened):
        handle = sim.world.get_object(obj.name)
        self._ensure_ur5e_primitive_control_ready(handle)
        velocity = (
            handle.metadata["open_gripper_velocity"]
            if opened
            else handle.metadata["closed_gripper_velocity"]
        )
        indices = handle.metadata["gripper_dof_indices"]
        handle.wrapper.switch_dof_control_mode("velocity", dof_indices=indices)
        handle.wrapper.set_dof_velocity_targets(
            np.full((1, len(indices)), float(velocity), dtype=float),
            dof_indices=indices,
        )

    def set_ur5e_arm_joint_positions(self, sim, obj, joint_positions):
        handle = sim.world.get_object(obj.name)
        self._ensure_ur5e_primitive_control_ready(handle)
        joint_positions = np.asarray(joint_positions, dtype=float).reshape(-1)
        arm_dof_indices = handle.metadata["arm_dof_indices"]
        if len(joint_positions) > len(arm_dof_indices):
            raise RuntimeError("UR5e arm joint target has more than 6 positions")
        handle.wrapper.set_dof_position_targets(
            joint_positions.tolist(),
            dof_indices=arm_dof_indices[: len(joint_positions)],
        )

    def hold_ur5e_position(self, sim, obj):
        handle = sim.world.get_object(obj.name)
        self._ensure_ur5e_primitive_control_ready(handle)
        arm_dof_indices = handle.metadata["arm_dof_indices"]
        current_dof_positions = handle.wrapper.get_dof_positions().numpy()
        arm_targets = current_dof_positions[:, arm_dof_indices].reshape(-1).tolist()
        handle.wrapper.set_dof_position_targets(
            arm_targets,
            dof_indices=arm_dof_indices,
        )

    def get_ur5e_end_effector_pose(self, sim, obj):
        handle = sim.world.get_object(obj.name)
        position, orientation = self._ur5e_end_effector_pose(handle)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return (
            self._tcp_position(handle, position, orientation),
            orientation,
        )

    def get_ur5e_gripper_positions(self, sim, obj):
        handle = sim.world.get_object(obj.name)
        gripper_dof_indices = handle.metadata["gripper_dof_indices"]
        dof_positions = handle.wrapper.get_dof_positions().numpy()
        return dof_positions[:, gripper_dof_indices].reshape(-1)

    def ur5e_gripper_target_positions(self, opened):
        if opened:
            return np.array([UR5E_GRIPPER_OPEN_POSITION], dtype=float)
        return np.array([UR5E_GRIPPER_CLOSED_POSITION], dtype=float)

    def get_physics_properties(self, world, obj):
        handle = world.get_object(obj.name)
        position, orientation = handle.wrapper.get_world_poses()
        position = position.numpy()[0]
        orientation = orientation.numpy()[0]
        yaw, pitch, roll = self.isaac_quat_to_scenic_euler_angles(orientation)
        linear_velocity, angular_velocity = handle.wrapper.get_velocities()
        linear_velocity = linear_velocity.numpy()[0]
        angular_velocity = angular_velocity.numpy()[0]
        lx, ly, lz = linear_velocity
        ax, ay, az = angular_velocity
        return {
            "position": tuple(position),
            "velocity": (lx, ly, lz),
            "speed": math.hypot(lx, ly, lz),
            "angularSpeed": math.hypot(ax, ay, az),
            "angularVelocity": (ax, ay, az),
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
        }

    def _apply_articulation_action(self, articulation, action):
        if not isinstance(action, IsaacArticulationAction):
            if isinstance(action, dict):
                action = IsaacArticulationAction(action)
            else:
                raise TypeError(
                    f"{self.name} robot controls must return articulation_action(...) "
                    "or a dict with joint position/velocity/effort fields"
                )
        kwargs = action.kwargs
        dof_indices = kwargs.get("joint_indices", kwargs.get("dof_indices"))
        if "joint_positions" in kwargs:
            articulation.set_dof_position_targets(
                kwargs["joint_positions"], dof_indices=dof_indices
            )
        if "joint_velocities" in kwargs:
            articulation.set_dof_velocity_targets(
                kwargs["joint_velocities"], dof_indices=dof_indices
            )
        if "joint_efforts" in kwargs:
            articulation.set_dof_efforts(kwargs["joint_efforts"], dof_indices=dof_indices)

    def _dof_indices(self, articulation, names):
        dof_names = list(articulation.dof_names)
        missing = [name for name in names if name not in dof_names]
        if missing:
            raise RuntimeError(f"{articulation.paths[0]} is missing required DOFs: {missing}")
        return [dof_names.index(name) for name in names]
