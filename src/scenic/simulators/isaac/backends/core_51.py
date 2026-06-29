import math
import os

import numpy as np

from scenic.simulators.isaac.backends.base import IsaacBackend
import scenic.simulators.isaac.utils as scenic_utils


def _as_array(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=float)


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
UR5E_RMPFLOW_DOWNWARD_ORIENTATION = np.array(
    [0.0, 0.70710678, 0.70710678, 0.0],
    dtype=float,
)
UR5E_RMPFLOW_TCP_OFFSET = np.array([0.0, 0.0, 0.135], dtype=float)
UR5E_GRIPPER_OPEN_POSITION = 0.0
UR5E_GRIPPER_CLOSED_POSITION = np.deg2rad(40.0)
UR5E_GRIPPER_FULLY_CLOSED_POSITION = 47.0
UR5E_GRIPPER_CLOSE_VELOCITY = np.deg2rad(90.0)
UR5E_GRIPPER_OPEN_VELOCITY = -np.deg2rad(45.0)
UR5E_GRIPPER_MAX_FORCE = 5.0
UR5E_GRIPPER_STIFFNESS = 0.0
UR5E_GRIPPER_DAMPING = 5000.0
UR5E_GRIPPER_MAX_JOINT_VELOCITY_DEG_PER_SEC = 130.0
UR5E_MIMIC_NATURAL_FREQUENCY = 0.0
UR5E_MIMIC_DAMPING_RATIO = 0.0
UR5E_OUTER_FINGER_PARALLEL_STIFFNESS = 0.05
UR5E_GRIPPER_VARIANT = "Robotiq_2f_85"
UR5E_GRIPPER_PRIM = "Gripper/Robotiq_2F_85"


class _AckermannControllerAdapter:
    def __init__(self, controller):
        self.controller = controller

    def forward(self, command):
        action = self.controller.forward(command=command)
        return action.joint_positions, action.joint_velocities


class Core51Backend(IsaacBackend):
    """Isaac Sim 5.1.0 backend implemented with the current Core API."""

    name = "core_51"

    def create_world(self, timestep):
        from isaacsim.core.api import World

        self._physics_dt = timestep
        return World(
            stage_units_in_meters=1.0,
            physics_dt=timestep,
            rendering_dt=timestep,
        )

    def open_environment_stage(self, usd_path):
        from isaacsim.core.utils import stage as stage_utils

        opened = stage_utils.open_stage(usd_path)
        if not opened:
            return False

        stage = stage_utils.get_current_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        return True

    def enable_extension(self, name):
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension(name)

    def initialize_physics(self, world, objects):
        world.initialize_physics()
        for obj in objects:
            if not obj.physics:
                continue
            isaac_obj = world.scene.get_object(obj.name)
            if hasattr(isaac_obj, "initialize"):
                isaac_obj.initialize()
            if getattr(obj, "wheel_controller", None) in {
                "differential",
                "holonomic",
                "ackermann",
            }:
                obj.wheel_dof_indices = list(isaac_obj.wheel_dof_indices)
                if obj.wheel_controller == "ackermann":
                    obj.steering_dof_indices = [
                        isaac_obj.get_dof_index(name) for name in obj.steering_dof_names
                    ]

    def play_world(self, world):
        world.play()

    def step_world(self, world):
        world.step()

    def stop_and_clear_world(self, world):
        world.stop()
        world.clear()
        world.reset()

    def release_world(self, world):
        from isaacsim.core.api import World

        World.clear_instance()

    def add_object(self, world, obj, *, scenic_obj=None):
        world.scene.add(obj)

    def ensure_environment_mesh_paths(
        self,
        environment_usd_path,
        environment_mesh_path=None,
        environment_info_path=None,
        *,
        headless=True,
        overwrite=False,
    ):
        default_mesh_path, default_info_path = (
            scenic_utils.default_environment_mesh_paths(environment_usd_path)
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
        )
        return mesh_path, info_path

    def apply_visual_material(self, wrapper, obj):
        from isaacsim.core.api.materials import PreviewSurface

        material = PreviewSurface(
            prim_path=f"/World/material/{obj.name}",
            color=scenic_utils.colorToArray(obj.color),
        )
        wrapper.apply_visual_material(material)

    def disable_rigid_body(self, prim):
        from pxr import Usd, UsdPhysics

        for descendant in Usd.PrimRange(prim):
            if descendant.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_api = UsdPhysics.RigidBodyAPI(descendant)
                rigid_body_api.CreateRigidBodyEnabledAttr(False)

    def create_controller(self, forward_func, name):
        from isaacsim.core.api.controllers import BaseController

        class Controller(BaseController):
            def __init__(self):
                super().__init__(name=name)

            def forward(self, command):
                return forward_func(command)

        return Controller()

    def create_generic_object(self, obj):
        from isaacsim.core.prims import SingleGeometryPrim, SingleRigidPrim
        from isaacsim.core.utils import prims
        from omni.physx.scripts import utils as physx_utils

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )

        usd_prim = prims.create_prim(prim_path=prim_path, usd_path=usd_path)
        scenic_position = scenic_utils.vectorToArray(obj.position)
        orientation = self.scenic_to_isaac_orientation(obj.orientation)

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
            physx_utils.setRigidBody(
                prims.get_prim_at_path(prim_path), "convexDecomposition", False
            )
            wrapper = SingleRigidPrim(
                prim_path=prim_path,
                name=obj.name,
                position=root_position,
                orientation=orientation,
                mass=obj.mass,
                density=obj.density,
                linear_velocity=scenic_utils.vectorToArray(obj.velocity),
            )
        else:
            wrapper = SingleGeometryPrim(
                prim_path=prim_path,
                name=obj.name,
                position=root_position,
                orientation=orientation,
                collision=True,
            )
            self.disable_rigid_body(usd_prim)

        wrapper.set_world_pose(position=root_position, orientation=orientation)
        wrapper.set_local_scale(local_scale)

        if obj.color:
            self.apply_visual_material(wrapper, obj)
        return wrapper

    def create_robot(self, obj):
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage

        if getattr(obj, "wheel_controller", None) in {
            "differential",
            "holonomic",
            "ackermann",
        }:
            return self.create_wheeled_robot(obj)

        if obj.control:
            obj.controller = self.create_controller(obj.control, f"{obj.name}_controller")

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )
        add_reference_to_stage(usd_path, prim_path)
        return Robot(
            prim_path=prim_path,
            name=obj.name,
            position=scenic_utils.vectorToArray(obj.position),
            orientation=self.scenic_to_isaac_orientation(
                obj.orientation, initial_rotation=obj.initial_rotation
            ),
        )

    def create_wheeled_robot(self, obj):
        from isaacsim.robot.wheeled_robots.controllers import (
            AckermannController,
            DifferentialController,
            HolonomicController,
        )
        from isaacsim.robot.wheeled_robots.robots import (
            HolonomicRobotUsdSetup,
            WheeledRobot,
        )

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )
        wrapper = WheeledRobot(
            prim_path=prim_path,
            name=obj.name,
            wheel_dof_names=obj.wheel_dof_names,
            create_robot=True,
            usd_path=usd_path,
            position=scenic_utils.vectorToArray(obj.position),
            orientation=self.scenic_to_isaac_orientation(
                obj.orientation, initial_rotation=obj.initial_rotation
            ),
        )

        if obj.wheel_controller == "differential":
            obj.controller = DifferentialController(
                name=f"{obj.name}_controller",
                wheel_radius=obj.wheel_radius,
                wheel_base=obj.wheel_base,
            )
        elif obj.wheel_controller == "holonomic":
            holonomic_setup = HolonomicRobotUsdSetup(
                robot_prim_path=prim_path,
                com_prim_path=f"{prim_path}/base_link/control_offset",
            )
            (
                wheel_radius,
                wheel_positions,
                wheel_orientations,
                mecanum_angles,
                wheel_axis,
                up_axis,
            ) = holonomic_setup.get_holonomic_controller_params()
            obj.controller = HolonomicController(
                name=f"{obj.name}_controller",
                wheel_radius=wheel_radius,
                wheel_positions=wheel_positions,
                wheel_orientations=wheel_orientations,
                mecanum_angles=mecanum_angles,
                wheel_axis=wheel_axis,
                up_axis=up_axis,
                max_linear_speed=getattr(obj, "max_linear_speed", 0.5),
                max_angular_speed=getattr(obj, "max_angular_speed", 0.8),
                max_wheel_speed=getattr(obj, "max_wheel_speed", 10.0),
            )
        elif obj.wheel_controller == "ackermann":
            steering_dof_names = getattr(obj, "steering_dof_names", None)
            if not steering_dof_names:
                raise ValueError(
                    f"Ackermann robot {obj.name} requires steering_dof_names, "
                    "usually [front_left_steering_joint, front_right_steering_joint]."
                )

            obj.steering_dof_names = steering_dof_names
            front_wheel_radius = getattr(obj, "front_wheel_radius", obj.wheel_radius)
            back_wheel_radius = getattr(obj, "back_wheel_radius", obj.wheel_radius)
            obj.controller = _AckermannControllerAdapter(
                AckermannController(
                    name=f"{obj.name}_controller",
                    wheel_base=obj.wheel_base,
                    track_width=obj.track_width,
                    front_wheel_radius=front_wheel_radius,
                    back_wheel_radius=back_wheel_radius,
                )
            )

        if obj.color:
            self.apply_visual_material(wrapper, obj)
        return wrapper

    def create_franka_panda(self, obj):
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.robot.manipulators import SingleManipulator
        from isaacsim.robot.manipulators.grippers import ParallelGripper

        prim_path = f"/World/{obj.name}"
        robot_prim = add_reference_to_stage(
            usd_path=self.asset_path(
                "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
            ),
            prim_path=prim_path,
        )
        robot_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
        robot_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")

        gripper = ParallelGripper(
            end_effector_prim_path=f"{prim_path}/panda_rightfinger",
            joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
            joint_opened_positions=np.array([0.05, 0.05]),
            joint_closed_positions=np.array([0.01, 0.01]),
            action_deltas=np.array([0.01, 0.01]),
        )
        wrapper = SingleManipulator(
            prim_path=prim_path,
            name=obj.name,
            end_effector_prim_path=f"{prim_path}/panda_rightfinger",
            gripper=gripper,
        )
        wrapper.gripper.set_default_state(wrapper.gripper.joint_opened_positions)
        return wrapper

    def create_ur5e(self, obj):
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.robot.manipulators import SingleManipulator
        from isaacsim.robot.manipulators.grippers import ParallelGripper

        prim_path = f"/World/{obj.name}"
        base_position = self._ur5e_base_position(obj)
        robot_position = base_position
        robot_prim = add_reference_to_stage(
            usd_path=self.asset_path("Isaac/Robots/UniversalRobots/ur5e/ur5e.usd"),
            prim_path=prim_path,
        )
        self._set_required_variant(robot_prim, "Gripper", UR5E_GRIPPER_VARIANT)

        stage = get_current_stage()
        end_effector_prim_path = f"{prim_path}/{UR5E_GRIPPER_PRIM}/base_link"
        self._require_stage_prim(stage, end_effector_prim_path)
        self._configure_ur5e_gripper_attachment(stage, prim_path)
        self._configure_ur5e_default_joint_pose(stage, prim_path)
        self._configure_ur5e_closed_loop_gripper(stage, prim_path)
        self._configure_ur5e_gripper_drive(stage, prim_path)

        gripper = ParallelGripper(
            end_effector_prim_path=end_effector_prim_path,
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([UR5E_GRIPPER_OPEN_POSITION]),
            joint_closed_positions=np.array([UR5E_GRIPPER_CLOSED_POSITION]),
            use_mimic_joints=True,
        )
        wrapper = SingleManipulator(
            prim_path=prim_path,
            name=obj.name,
            position=robot_position,
            orientation=self.scenic_to_isaac_orientation(obj.orientation),
            end_effector_prim_path=end_effector_prim_path,
            gripper=gripper,
        )
        wrapper.gripper.set_default_state(wrapper.gripper.joint_opened_positions)
        return wrapper

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

        joint = self._require_stage_prim(stage, f"{prim_path}/joints/robot_gripper_joint")

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
            raise RuntimeError(
                f"Missing Robotiq finger_joint under {gripper_root.GetPath()}"
            )

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

    def create_ground_plane(self, obj):
        from isaacsim.core.api.objects import GroundPlane

        return GroundPlane(
            name=obj.name,
            prim_path="/World/GroundPlane",
            z_position=0,
            size=max(obj.width, obj.length),
            color=scenic_utils.colorToArray(obj.color),
        )

    def apply_robot_control(self, sim, obj, command):
        robot = sim.world.scene.get_object(obj.name)
        if obj.controller is None:
            return
        if getattr(obj, "wheel_controller", None) in {
            "differential",
            "holonomic",
            "ackermann",
        }:
            self.apply_wheeled_control(sim, obj, command)
            return

        action = obj.controller.forward(command=command)
        robot.apply_action(self._to_core_articulation_action(action))

    def apply_wheeled_control(self, sim, obj, command):
        wheeled_robot = sim.world.scene.get_object(obj.name)
        if obj.controller is None:
            return

        if obj.wheel_controller in {"differential", "holonomic"}:
            wheeled_robot.apply_wheel_actions(obj.controller.forward(command=command))
            return

        if obj.wheel_controller == "ackermann":
            from isaacsim.core.utils.types import ArticulationAction

            steering_positions, wheel_velocities = obj.controller.forward(command)
            wheeled_robot.apply_action(
                ArticulationAction(
                    joint_positions=steering_positions,
                    joint_indices=obj.steering_dof_indices,
                )
            )
            wheeled_robot.apply_wheel_actions(
                ArticulationAction(joint_velocities=wheel_velocities)
            )
            return

        action = obj.controller.forward(command=command)
        wheeled_robot.apply_action(self._to_core_articulation_action(action))

    def apply_articulation_action(self, sim, obj, action):
        robot = sim.world.scene.get_object(obj.name)
        for field, index_field in (
            ("joint_positions", "joint_position_indices"),
            ("joint_velocities", "joint_velocity_indices"),
            ("joint_efforts", "joint_effort_indices"),
        ):
            split_action = self._core_action_for_field(action, field, index_field)
            if split_action is not None:
                robot.apply_action(split_action)

    def _core_action_for_field(self, action, field, index_field):
        values = action.get(field)
        if values is None:
            return None

        from isaacsim.core.utils.types import ArticulationAction

        indices = action.get(
            index_field,
            action.get("joint_indices", action.get("dof_indices")),
        )
        return ArticulationAction(**{field: values, "joint_indices": indices})

    def articulation_dof_names(self, sim, obj):
        robot = sim.world.scene.get_object(obj.name)
        names = getattr(robot, "dof_names", None)
        if names is not None:
            return list(names)
        names = getattr(robot, "_dof_names", None)
        if names is not None:
            return list(names)
        raise RuntimeError(f"unable to read DOF names for {obj.name}")

    def get_object_pose(self, sim, obj):
        wrapper = sim.world.scene.get_object(obj.name)
        position, orientation = wrapper.get_world_pose()
        return np.array(position, dtype=float), np.array(orientation, dtype=float)

    def set_object_pose(self, sim, obj, position, orientation=None):
        wrapper = sim.world.scene.get_object(obj.name)
        position = np.array(position, dtype=float)
        if orientation is None:
            _, orientation = wrapper.get_world_pose()
        orientation = np.array(orientation, dtype=float)
        wrapper.set_world_pose(position=position, orientation=orientation)
        if hasattr(wrapper, "set_linear_velocity"):
            wrapper.set_linear_velocity(np.zeros(3, dtype=float))
        if hasattr(wrapper, "set_angular_velocity"):
            wrapper.set_angular_velocity(np.zeros(3, dtype=float))

    def move_franka_pick_place(
        self,
        sim,
        obj,
        target_object,
        goal_position,
        end_effector_offset=None,
        end_effector_orientation=None,
    ):
        franka = sim.world.scene.get_object(obj.name)
        if obj.controller is None:
            from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import (
                PickPlaceController,
            )

            obj.controller = PickPlaceController(
                name=f"{obj.name}_pick_place_controller",
                gripper=franka.gripper,
                robot_articulation=franka,
            )
            obj.controller.reset()
            franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
        if obj.controller.is_done():
            return

        target = sim.world.scene.get_object(target_object.name)
        picking_position, _ = target.get_world_pose()
        placing_position = scenic_utils.vectorToArray(goal_position)
        if end_effector_offset is None:
            end_effector_offset = obj.end_effector_offset
        if end_effector_orientation is None:
            end_effector_orientation = obj.end_effector_orientation
        offset = np.array(end_effector_offset, dtype=float)
        actions = obj.controller.forward(
            picking_position=picking_position,
            placing_position=placing_position,
            current_joint_positions=franka.get_joint_positions(),
            end_effector_offset=offset,
            end_effector_orientation=end_effector_orientation,
        )
        franka.apply_action(actions)

    def _motion_policy_state(self, sim, obj, manipulator, robot_name):
        states = getattr(obj, "_core_motion_policy_states", None)
        if states is None:
            states = {}
            obj._core_motion_policy_states = states
        if robot_name not in states:
            import isaacsim.robot_motion.motion_generation as mg

            config = mg.interface_config_loader.load_supported_motion_policy_config(
                robot_name, "RMPflow"
            )
            if config is None:
                raise RuntimeError(f"{robot_name} has no supported RMPflow config")
            rmp_flow = mg.lula.motion_policies.RmpFlow(**config)
            self._sync_motion_policy_base(manipulator, rmp_flow)
            policy = mg.ArticulationMotionPolicy(
                manipulator,
                rmp_flow,
                sim.timestep,
            )
            states[robot_name] = {
                "controller": mg.MotionPolicyController(
                    name=f"{obj.name}_{robot_name}_rmpflow",
                    articulation_motion_policy=policy,
                ),
                "policy": policy,
                "rmp_flow": rmp_flow,
            }
        return states[robot_name]

    def _sync_motion_policy_base(self, manipulator, rmp_flow):
        position, orientation = manipulator.get_world_pose()
        rmp_flow.set_robot_base_pose(
            robot_position=position,
            robot_orientation=orientation,
        )

    def _franka(self, sim, obj):
        franka = sim.world.scene.get_object(obj.name)
        if not getattr(obj, "_core_franka_primitive_ready", False):
            franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)
            obj._core_franka_primitive_ready = True
        return franka

    def move_franka_end_effector(self, sim, obj, position, orientation=None):
        franka = self._franka(sim, obj)
        state = self._motion_policy_state(sim, obj, franka, "Franka")
        self._sync_motion_policy_base(franka, state["rmp_flow"])
        if orientation is None:
            orientation = np.array([0.0, 1.0, 0.0, 0.0], dtype=float)
        action = state["controller"].forward(
            target_end_effector_position=_position_array(position),
            target_end_effector_orientation=np.asarray(orientation, dtype=float),
        )
        franka.apply_action(action)

    def set_franka_gripper(self, sim, obj, opened):
        franka = self._franka(sim, obj)
        action = franka.gripper.forward(action="open" if opened else "close")
        franka.apply_action(action)

    def set_franka_arm_joint_positions(self, sim, obj, joint_positions):
        franka = self._franka(sim, obj)
        joints = np.asarray(joint_positions, dtype=float).reshape(-1)
        targets = [None] * franka.num_dof
        for index, value in enumerate(joints[:7]):
            targets[index] = value
        from isaacsim.core.utils.types import ArticulationAction

        franka.apply_action(ArticulationAction(joint_positions=targets))

    def hold_franka_position(self, sim, obj):
        franka = self._franka(sim, obj)
        current = np.asarray(franka.get_joint_positions(), dtype=float)
        targets = [None] * franka.num_dof
        for index in range(min(7, len(current))):
            targets[index] = current[index]
        from isaacsim.core.utils.types import ArticulationAction

        franka.apply_action(ArticulationAction(joint_positions=targets))

    def get_franka_end_effector_pose(self, sim, obj):
        franka = self._franka(sim, obj)
        state = self._motion_policy_state(sim, obj, franka, "Franka")
        self._sync_motion_policy_base(franka, state["rmp_flow"])
        active_joints = state["policy"].get_active_joints_subset().get_joint_positions()
        position, orientation = state["rmp_flow"].get_end_effector_pose(active_joints)
        position = _as_array(position).reshape(-1)[:3]
        orientation = _as_array(orientation)
        if orientation.shape == (3, 3):
            from isaacsim.core.utils.rotations import rot_matrix_to_quat

            orientation = rot_matrix_to_quat(orientation)
        return position, orientation.reshape(-1)[:4]

    def get_franka_gripper_positions(self, sim, obj):
        franka = self._franka(sim, obj)
        return np.asarray(franka.gripper.get_joint_positions(), dtype=float)

    def franka_gripper_target_positions(self, opened):
        if opened:
            return np.array([0.05, 0.05], dtype=float)
        return np.array([0.01, 0.01], dtype=float)

    def _core_dof_indices(self, articulation, names):
        dof_names = list(articulation.dof_names)
        missing = [name for name in names if name not in dof_names]
        if missing:
            raise RuntimeError(f"{articulation.name} is missing required DOFs: {missing}")
        return [dof_names.index(name) for name in names]

    def _tcp_to_control_position(self, tcp_position, orientation, tcp_offset):
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return _position_array(tcp_position) - self.rotate_vector_by_wxyz_quat(
            orientation, tcp_offset
        )

    def _control_to_tcp_position(self, control_position, orientation, tcp_offset):
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        control_position = np.asarray(control_position, dtype=float).reshape(-1)[:3]
        return control_position + self.rotate_vector_by_wxyz_quat(orientation, tcp_offset)

    def _ur5e(self, sim, obj):
        ur5e = sim.world.scene.get_object(obj.name)
        if not hasattr(obj, "_core_ur5e_primitive_ready"):
            arm_dof_indices = self._core_dof_indices(ur5e, UR5E_ARM_DOF_NAMES)
            ur5e.set_joint_positions(
                UR5E_DEFAULT_ARM_POSE,
                joint_indices=np.asarray(arm_dof_indices, dtype=np.int32),
            )
            ur5e.gripper.set_joint_positions(ur5e.gripper.joint_opened_positions)
            obj._core_ur5e_primitive_ready = True
        return ur5e

    def move_ur5e_end_effector(self, sim, obj, position, orientation=None):
        ur5e = self._ur5e(sim, obj)
        state = self._motion_policy_state(sim, obj, ur5e, "UR5e")
        self._sync_motion_policy_base(ur5e, state["rmp_flow"])
        if orientation is None:
            orientation = UR5E_RMPFLOW_DOWNWARD_ORIENTATION
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        control_position = self._tcp_to_control_position(
            position, orientation, UR5E_RMPFLOW_TCP_OFFSET
        )
        action = state["controller"].forward(
            target_end_effector_position=control_position,
            target_end_effector_orientation=orientation,
        )
        ur5e.apply_action(action)

    def set_ur5e_gripper(self, sim, obj, opened):
        ur5e = self._ur5e(sim, obj)
        velocity = UR5E_GRIPPER_OPEN_VELOCITY if opened else UR5E_GRIPPER_CLOSE_VELOCITY
        indices = self._core_dof_indices(ur5e, ["finger_joint"])
        ur5e.get_articulation_controller().switch_dof_control_mode(
            dof_index=indices[0],
            mode="velocity",
        )
        from isaacsim.core.utils.types import ArticulationAction

        action = ArticulationAction(
            joint_velocities=np.array([velocity], dtype=float),
            joint_indices=np.asarray(indices, dtype=np.int32),
        )
        ur5e.apply_action(action)

    def set_ur5e_arm_joint_positions(self, sim, obj, joint_positions):
        ur5e = self._ur5e(sim, obj)
        joints = np.asarray(joint_positions, dtype=float).reshape(-1)
        arm_dof_indices = self._core_dof_indices(ur5e, UR5E_ARM_DOF_NAMES)
        if len(joints) > len(arm_dof_indices):
            raise RuntimeError("UR5e arm joint target has more than 6 positions")
        targets = [None] * ur5e.num_dof
        for index, value in zip(arm_dof_indices, joints):
            targets[index] = value
        from isaacsim.core.utils.types import ArticulationAction

        ur5e.apply_action(ArticulationAction(joint_positions=targets))

    def hold_ur5e_position(self, sim, obj):
        ur5e = self._ur5e(sim, obj)
        arm_dof_indices = self._core_dof_indices(ur5e, UR5E_ARM_DOF_NAMES)
        current = np.asarray(ur5e.get_joint_positions(), dtype=float)
        targets = [None] * ur5e.num_dof
        for index in arm_dof_indices:
            targets[index] = current[index]
        from isaacsim.core.utils.types import ArticulationAction

        ur5e.apply_action(ArticulationAction(joint_positions=targets))

    def get_ur5e_end_effector_pose(self, sim, obj):
        ur5e = self._ur5e(sim, obj)
        state = self._motion_policy_state(sim, obj, ur5e, "UR5e")
        self._sync_motion_policy_base(ur5e, state["rmp_flow"])
        active_joints = state["policy"].get_active_joints_subset().get_joint_positions()
        position, orientation = state["rmp_flow"].get_end_effector_pose(active_joints)
        position = _as_array(position).reshape(-1)[:3]
        orientation = _as_array(orientation)
        if orientation.shape == (3, 3):
            from isaacsim.core.utils.rotations import rot_matrix_to_quat

            orientation = rot_matrix_to_quat(orientation)
        orientation = orientation.reshape(-1)[:4]
        return (
            self._control_to_tcp_position(
                position,
                orientation,
                UR5E_RMPFLOW_TCP_OFFSET,
            ),
            orientation,
        )

    def get_ur5e_gripper_positions(self, sim, obj):
        ur5e = self._ur5e(sim, obj)
        positions = np.asarray(ur5e.gripper.get_joint_positions(), dtype=float)
        return positions.reshape(-1)

    def ur5e_gripper_target_positions(self, opened):
        if opened:
            return np.array([UR5E_GRIPPER_OPEN_POSITION], dtype=float)
        return np.array([UR5E_GRIPPER_CLOSED_POSITION], dtype=float)

    def get_physics_properties(self, world, obj):
        isaac_obj = world.scene.get_object(obj.name)
        position, orientation = isaac_obj.get_world_pose()
        x, y, z = position
        yaw, pitch, roll = self.isaac_quat_to_scenic_euler_angles(orientation)
        lx, ly, lz = isaac_obj.get_linear_velocity()
        ax, ay, az = isaac_obj.get_angular_velocity()
        return {
            "position": (x, y, z),
            "velocity": (lx, ly, lz),
            "speed": math.hypot(lx, ly, lz),
            "angularSpeed": math.hypot(ax, ay, az),
            "angularVelocity": (ax, ay, az),
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
        }

    def _to_core_articulation_action(self, action):
        from isaacsim.core.utils.types import ArticulationAction

        return ArticulationAction(**action)
