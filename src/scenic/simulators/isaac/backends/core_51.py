import math
import os

import numpy as np

from scenic.simulators.isaac.backends.base import IsaacBackend
import scenic.simulators.isaac.utils as scenic_utils


def _asArray(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=float)


def _positionArray(position):
    if hasattr(position, "x") and hasattr(position, "y") and hasattr(position, "z"):
        return np.array([position.x, position.y, position.z], dtype=float)
    return np.asarray(position, dtype=float).reshape(-1)[:3]


class _AckermannControllerAdapter:
    def __init__(self, controller):
        self.controller = controller

    def forward(self, command):
        action = self.controller.forward(command=command)
        return action.joint_positions, action.joint_velocities


class Core51Backend(IsaacBackend):
    """Isaac Sim 5.1.0 backend implemented with the current Core API."""

    name = "core_51"

    def createWorld(self, timestep):
        from isaacsim.core.api import World

        self._physics_dt = timestep
        return World(
            stage_units_in_meters=1.0,
            physics_dt=timestep,
            rendering_dt=timestep,
        )

    def openEnvironmentStage(self, usd_path):
        from isaacsim.core.utils import stage as stage_utils

        opened = stage_utils.open_stage(usd_path)
        if not opened:
            return False

        stage = stage_utils.get_current_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        return True

    def enableExtension(self, name):
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension(name)

    def initializePhysics(self, world, objects):
        world.initialize_physics()
        for obj in objects:
            if not obj.physics:
                continue
            isaac_obj = world.scene.get_object(obj.name)
            if hasattr(isaac_obj, "initialize"):
                isaac_obj.initialize()
            if getattr(obj, "wheelController", None) in {
                "differential",
                "holonomic",
                "ackermann",
            }:
                obj.wheelDofIndices = list(isaac_obj.wheel_dof_indices)
                if obj.wheelController == "ackermann":
                    obj.steeringDofIndices = [
                        isaac_obj.get_dof_index(name) for name in obj.steeringDofNames
                    ]

    def playWorld(self, world):
        world.play()

    def stepWorld(self, world):
        world.step()

    def stopAndClearWorld(self, world):
        world.stop()
        world.clear()
        world.reset()

    def releaseWorld(self, world):
        from isaacsim.core.api import World

        World.clear_instance()

    def addObject(self, world, obj, *, scenic_obj=None):
        world.scene.add(obj)

    def ensureEnvironmentMeshPaths(
        self,
        environmentUsdPath,
        environment_mesh_path=None,
        environment_info_path=None,
        *,
        headless=True,
        overwrite=False,
    ):
        default_mesh_path, default_info_path = scenic_utils.defaultEnvironmentMeshPaths(
            environmentUsdPath
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

        if not overwrite and scenic_utils.environmentOutputsCurrent(
            environmentUsdPath, mesh_path, info_path
        ):
            return mesh_path, info_path

        mesh_path.parent.mkdir(parents=True, exist_ok=True)
        info_path.parent.mkdir(parents=True, exist_ok=True)

        if not self.kitAppRunning():
            self.getSimulationApp(headless=headless)

        self.enableExtension("omni.kit.asset_converter")
        from scenic.simulators.isaac.backends.core_51_usd_to_mesh import (
            convertEnvironmentUsd,
        )

        convertEnvironmentUsd(
            self.kitUsdPath(environmentUsdPath),
            str(mesh_path),
            str(info_path),
            overwrite=True,
        )
        return mesh_path, info_path

    def applyVisualMaterial(self, wrapper, obj):
        from isaacsim.core.api.materials import PreviewSurface

        material = PreviewSurface(
            prim_path=f"/World/material/{obj.name}",
            color=scenic_utils.colorToArray(obj.color),
        )
        wrapper.apply_visual_material(material)

    def disableRigidBody(self, prim):
        from pxr import Usd, UsdPhysics

        for descendant in Usd.PrimRange(prim):
            if descendant.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_api = UsdPhysics.RigidBodyAPI(descendant)
                rigid_body_api.CreateRigidBodyEnabledAttr(False)

    def createController(self, forward_func, name):
        from isaacsim.core.api.controllers import BaseController

        class Controller(BaseController):
            def __init__(self):
                super().__init__(name=name)

            def forward(self, command):
                return forward_func(command)

        return Controller()

    def createGenericObject(self, obj):
        from isaacsim.core.prims import SingleGeometryPrim, SingleRigidPrim
        from isaacsim.core.utils import prims
        from omni.physx.scripts import utils as physx_utils

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.assetPath(obj.isaacAssetPath)
            if obj.isaacAssetPath
            else os.path.abspath(obj.usdPath)
        )

        usd_prim = prims.create_prim(prim_path=prim_path, usd_path=usd_path)
        scenic_position = scenic_utils.vectorToArray(obj.position)
        orientation = self.scenicToIsaacOrientation(obj.orientation)

        # Compute scale from Scenic dimensions to native USD dimensions.
        root_position, local_scale, native_size, native_center = (
            self.computeUsdScaleAndRootPosition(
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
            self.disableRigidBody(usd_prim)

        wrapper.set_world_pose(position=root_position, orientation=orientation)
        wrapper.set_local_scale(local_scale)

        if obj.color:
            self.applyVisualMaterial(wrapper, obj)
        return wrapper

    def createRobot(self, obj):
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage

        if getattr(obj, "manipulatorProfile", None) is not None:
            return self.createManipulator(obj)

        if getattr(obj, "wheelController", None) in {
            "differential",
            "holonomic",
            "ackermann",
        }:
            return self.createWheeledRobot(obj)

        if obj.control:
            obj.controller = self.createController(obj.control, f"{obj.name}_controller")

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.assetPath(obj.isaacAssetPath)
            if obj.isaacAssetPath
            else os.path.abspath(obj.usdPath)
        )
        add_reference_to_stage(usd_path, prim_path)
        return Robot(
            prim_path=prim_path,
            name=obj.name,
            position=scenic_utils.vectorToArray(obj.position),
            orientation=self.scenicToIsaacOrientation(
                obj.orientation, initial_rotation=obj.initialRotation
            ),
        )

    def createWheeledRobot(self, obj):
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
            self.assetPath(obj.isaacAssetPath)
            if obj.isaacAssetPath
            else os.path.abspath(obj.usdPath)
        )
        wrapper = WheeledRobot(
            prim_path=prim_path,
            name=obj.name,
            wheel_dof_names=obj.wheelDofNames,
            create_robot=True,
            usd_path=usd_path,
            position=scenic_utils.vectorToArray(obj.position),
            orientation=self.scenicToIsaacOrientation(
                obj.orientation, initial_rotation=obj.initialRotation
            ),
        )

        if obj.wheelController == "differential":
            obj.controller = DifferentialController(
                name=f"{obj.name}_controller",
                wheel_radius=obj.wheelRadius,
                wheel_base=obj.wheelBase,
            )
        elif obj.wheelController == "holonomic":
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
                max_linear_speed=getattr(obj, "maxLinearSpeed", 0.5),
                max_angular_speed=getattr(obj, "maxAngularSpeed", 0.8),
                max_wheel_speed=getattr(obj, "maxWheelSpeed", 10.0),
            )
        elif obj.wheelController == "ackermann":
            steering_dof_names = getattr(obj, "steeringDofNames", None)
            if not steering_dof_names:
                raise ValueError(
                    f"Ackermann robot {obj.name} requires steering_dof_names, "
                    "usually [front_left_steering_joint, front_right_steering_joint]."
                )

            obj.steeringDofNames = steering_dof_names
            front_wheel_radius = getattr(obj, "frontWheelRadius", obj.wheelRadius)
            back_wheel_radius = getattr(obj, "backWheelRadius", obj.wheelRadius)
            obj.controller = _AckermannControllerAdapter(
                AckermannController(
                    name=f"{obj.name}_controller",
                    wheel_base=obj.wheelBase,
                    track_width=obj.trackWidth,
                    front_wheel_radius=front_wheel_radius,
                    back_wheel_radius=back_wheel_radius,
                )
            )

        if obj.color:
            self.applyVisualMaterial(wrapper, obj)
        return wrapper

    def createManipulator(self, obj):
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.robot.manipulators import SingleManipulator
        from isaacsim.robot.manipulators.grippers import ParallelGripper

        profile = obj.manipulatorProfile
        prim_path = f"/World/{obj.name}"
        robot_prim = add_reference_to_stage(
            usd_path=self.kitUsdPath(profile.usdPath),
            prim_path=prim_path,
        )
        for variant_name, selection in profile.usdVariants:
            self._setRequiredVariant(robot_prim, variant_name, selection)

        stage = get_current_stage()
        end_effector_prim_path = f"{prim_path}/{profile.gripperFramePrim}"
        self._requireStagePrim(stage, end_effector_prim_path)

        if profile.gripperStyle == "robotiq_2f85":
            self._configureRobotiqGripperAttachment(stage, prim_path, profile)
            self._configureRobotiqDefaultJointPose(stage, prim_path, profile)
            self._configureRobotiqClosedLoopGripper(stage, prim_path, profile)
            self._configureRobotiqGripperDrive(stage, prim_path, profile)
            gripper = ParallelGripper(
                end_effector_prim_path=end_effector_prim_path,
                joint_prim_names=list(profile.gripperDofNames),
                joint_opened_positions=profile.openGripperPositions.copy(),
                joint_closed_positions=profile.closedGripperPositions.copy(),
                use_mimic_joints=True,
            )
        else:
            action_deltas = getattr(profile, "gripperActionDeltas", None)
            gripper = ParallelGripper(
                end_effector_prim_path=end_effector_prim_path,
                joint_prim_names=list(profile.gripperDofNames),
                joint_opened_positions=profile.openGripperPositions.copy(),
                joint_closed_positions=profile.closedGripperPositions.copy(),
                action_deltas=(
                    None
                    if action_deltas is None
                    else np.array(action_deltas, dtype=float)
                ),
            )
        wrapper = SingleManipulator(
            prim_path=prim_path,
            name=obj.name,
            position=self._manipulatorRootPosition(obj),
            orientation=self.scenicToIsaacOrientation(obj.orientation),
            end_effector_prim_path=end_effector_prim_path,
            gripper=gripper,
        )
        wrapper.gripper.set_default_state(wrapper.gripper.joint_opened_positions)
        return wrapper

    def _setRequiredVariant(self, prim, variant_name, selection):
        variant_set = prim.GetVariantSet(variant_name)
        if not variant_set or not variant_set.IsValid():
            raise RuntimeError(f"{prim.GetPath()} has no {variant_name!r} variant set")
        available = list(variant_set.GetVariantNames())
        if selection not in available:
            raise RuntimeError(
                f"{prim.GetPath()} {variant_name!r} variant {selection!r} is missing"
            )
        variant_set.SetVariantSelection(selection)

    def _requireStagePrim(self, stage, prim_path):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Required Isaac prim is missing: {prim_path}")
        return prim

    def _manipulatorRootPosition(self, obj):
        position = scenic_utils.vectorToArray(obj.position)
        position[2] -= obj.height / 2
        return position

    def _configureRobotiqGripperAttachment(self, stage, prim_path, profile):
        from pxr import Gf, Sdf

        joint = self._requireStagePrim(stage, f"{prim_path}/joints/robot_gripper_joint")

        def setQuatAttr(attr_name, values):
            attr = joint.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Quatf)
            attr.Set(
                Gf.Quatf(
                    float(values[0]),
                    Gf.Vec3f(float(values[1]), float(values[2]), float(values[3])),
                )
            )

        setQuatAttr("physics:localRot0", (0.70710677, 0.0, 0.0, 0.70710677))
        setQuatAttr("physics:localRot1", (1.0, 0.0, 0.0, 0.0))

    def _configureRobotiqDefaultJointPose(self, stage, prim_path, profile):
        from pxr import Sdf

        for joint_name, angle_deg in zip(
            profile.armDofNames, np.rad2deg(profile.defaultArmPose)
        ):
            joint = self._requireStagePrim(stage, f"{prim_path}/joints/{joint_name}")
            for attr_name in (
                "drive:angular:physics:targetPosition",
                "state:angular:physics:position",
            ):
                attr = joint.GetAttribute(attr_name)
                if not attr or not attr.IsValid():
                    attr = joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
                attr.Set(float(angle_deg))

        gripper_joint = self._requireStagePrim(
            stage, f"{prim_path}/{profile.gripperPrim}/Joints/finger_joint"
        )
        for attr_name in (
            "drive:angular:physics:targetPosition",
            "state:angular:physics:position",
        ):
            attr = gripper_joint.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = gripper_joint.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
            attr.Set(float(profile.openGripperPositions[0]))

    def _configureRobotiqGripperDrive(self, stage, prim_path, profile):
        from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

        gripper_root = self._requireStagePrim(stage, f"{prim_path}/{profile.gripperPrim}")
        found_finger_joint = False

        def setAttr(prim, attr_name, value, value_type):
            attr = prim.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute(attr_name, value_type)
            attr.Set(float(value))
            return attr

        def setDriveAttrs(
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
                float(profile.gripperMaxJointVelocityDegPerSec)
            )

        for prim in Usd.PrimRange(gripper_root):
            name = prim.GetName()
            if "Joint" not in str(prim.GetTypeName()) and not name.endswith("_joint"):
                continue
            for schema in list(prim.GetAppliedSchemas()):
                if not schema.startswith("PhysxMimicJointAPI:"):
                    continue
                axis = schema.split(":", 1)[1]
                setAttr(
                    prim,
                    f"physxMimicJoint:{axis}:naturalFrequency",
                    profile.mimicNaturalFrequency,
                    Sdf.ValueTypeNames.Float,
                )
                setAttr(
                    prim,
                    f"physxMimicJoint:{axis}:dampingRatio",
                    profile.mimicDampingRatio,
                    Sdf.ValueTypeNames.Float,
                )
            if name == "finger_joint":
                found_finger_joint = True
                setDriveAttrs(
                    prim,
                    profile.gripperMaxForce,
                    profile.gripperStiffness,
                    profile.gripperDamping,
                )
                setAttr(prim, "physics:lowerLimit", 0.0, Sdf.ValueTypeNames.Float)
                setAttr(
                    prim,
                    "physics:upperLimit",
                    profile.gripperFullyClosedPosition,
                    Sdf.ValueTypeNames.Float,
                )
            elif name in ("left_outer_finger_joint", "right_outer_finger_joint"):
                setDriveAttrs(
                    prim,
                    profile.gripperMaxForce,
                    profile.outerFingerParallelStiffness,
                    profile.gripperDamping,
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

    def _configureRobotiqClosedLoopGripper(self, stage, prim_path, profile):
        from pxr import Gf, PhysxSchema, Sdf, Usd, UsdPhysics

        base_path = f"{prim_path}/{profile.gripperPrim}/base_link"
        joint_root_path = f"{prim_path}/{profile.gripperPrim}/Joints"
        self._requireStagePrim(stage, base_path)
        joint_root = self._requireStagePrim(stage, joint_root_path)
        for body_path in (
            f"{prim_path}/{profile.gripperPrim}/left_inner_knuckle",
            f"{prim_path}/{profile.gripperPrim}/right_inner_knuckle",
        ):
            self._requireStagePrim(stage, body_path)

        def setAttr(prim, attr_name, value, value_type):
            attr = prim.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute(attr_name, value_type)
            attr.Set(value)
            return attr

        def configureJointCommon(prim, exclude_from_articulation):
            setAttr(
                prim,
                "physics:excludeFromArticulation",
                bool(exclude_from_articulation),
                Sdf.ValueTypeNames.Bool,
            )
            setAttr(prim, "physics:jointEnabled", True, Sdf.ValueTypeNames.Bool)
            if not prim.HasAPI(PhysxSchema.PhysxJointAPI):
                PhysxSchema.PhysxJointAPI.Apply(prim)

        passive_joint_specs = (
            (
                "left_inner_knuckle_joint",
                f"{prim_path}/{profile.gripperPrim}/left_inner_knuckle",
                (0.0, -0.0127, 0.06142),
                (0.5, 0.5, -0.5, -0.5),
            ),
            (
                "right_inner_knuckle_joint",
                f"{prim_path}/{profile.gripperPrim}/right_inner_knuckle",
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
            configureJointCommon(joint_prim, exclude_from_articulation=True)

        for prim in Usd.PrimRange(joint_root):
            if prim.GetName() in (
                "left_inner_finger_knuckle_joint",
                "right_inner_finger_knuckle_joint",
            ):
                configureJointCommon(prim, exclude_from_articulation=False)

    def createGroundPlane(self, obj):
        from isaacsim.core.api.objects import GroundPlane

        return GroundPlane(
            name=obj.name,
            prim_path="/World/GroundPlane",
            z_position=0,
            size=max(obj.width, obj.length),
            color=scenic_utils.colorToArray(obj.color),
        )

    def applyRobotControl(self, sim, obj, command):
        robot = sim.world.scene.get_object(obj.name)
        if obj.controller is None:
            return
        if getattr(obj, "wheelController", None) in {
            "differential",
            "holonomic",
            "ackermann",
        }:
            self.applyWheeledControl(sim, obj, command)
            return

        action = obj.controller.forward(command=command)
        robot.apply_action(self._toCoreArticulationAction(action))

    def applyWheeledControl(self, sim, obj, command):
        wheeled_robot = sim.world.scene.get_object(obj.name)
        if obj.controller is None:
            return

        if obj.wheelController in {"differential", "holonomic"}:
            wheeled_robot.apply_wheel_actions(obj.controller.forward(command=command))
            return

        if obj.wheelController == "ackermann":
            from isaacsim.core.utils.types import ArticulationAction

            steering_positions, wheel_velocities = obj.controller.forward(command)
            wheeled_robot.apply_action(
                ArticulationAction(
                    joint_positions=steering_positions,
                    joint_indices=obj.steeringDofIndices,
                )
            )
            wheeled_robot.apply_wheel_actions(
                ArticulationAction(joint_velocities=wheel_velocities)
            )
            return

        action = obj.controller.forward(command=command)
        wheeled_robot.apply_action(self._toCoreArticulationAction(action))

    def applyArticulationAction(self, sim, obj, action):
        robot = sim.world.scene.get_object(obj.name)
        for field, index_field in (
            ("joint_positions", "joint_position_indices"),
            ("joint_velocities", "joint_velocity_indices"),
            ("joint_efforts", "joint_effort_indices"),
        ):
            split_action = self._coreActionForField(action, field, index_field)
            if split_action is not None:
                robot.apply_action(split_action)

    def _coreActionForField(self, action, field, index_field):
        values = action.get(field)
        if values is None:
            return None

        from isaacsim.core.utils.types import ArticulationAction

        indices = action.get(
            index_field,
            action.get("joint_indices", action.get("dof_indices")),
        )
        return ArticulationAction(**{field: values, "joint_indices": indices})

    def articulationDofNames(self, sim, obj):
        robot = sim.world.scene.get_object(obj.name)
        names = getattr(robot, "dof_names", None)
        if names is not None:
            return list(names)
        names = getattr(robot, "_dof_names", None)
        if names is not None:
            return list(names)
        raise RuntimeError(f"unable to read DOF names for {obj.name}")

    def getObjectPose(self, sim, obj):
        wrapper = sim.world.scene.get_object(obj.name)
        position, orientation = wrapper.get_world_pose()
        return np.array(position, dtype=float), np.array(orientation, dtype=float)

    def setObjectPose(self, sim, obj, position, orientation=None):
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

    def moveManipulatorPickPlace(
        self,
        sim,
        obj,
        targetObject,
        goalPosition,
        endEffectorOffset=None,
        endEffectorOrientation=None,
    ):
        if not obj.manipulatorProfile.supportsPickPlace:
            raise RuntimeError(
                f"{type(obj).__name__} does not support built-in pick-place"
            )
        manipulator = sim.world.scene.get_object(obj.name)
        if obj.controller is None:
            from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import (
                PickPlaceController,
            )

            obj.controller = PickPlaceController(
                name=f"{obj.name}_pick_place_controller",
                gripper=manipulator.gripper,
                robot_articulation=manipulator,
            )
            obj.controller.reset()
            manipulator.gripper.set_joint_positions(
                manipulator.gripper.joint_opened_positions
            )
        if obj.controller.is_done():
            return

        target = sim.world.scene.get_object(targetObject.name)
        picking_position, _ = target.get_world_pose()
        placing_position = scenic_utils.vectorToArray(goalPosition)
        if endEffectorOffset is None:
            endEffectorOffset = obj.end_effector_offset
        if endEffectorOrientation is None:
            endEffectorOrientation = obj.end_effector_orientation
        offset = np.array(endEffectorOffset, dtype=float)
        actions = obj.controller.forward(
            picking_position=picking_position,
            placing_position=placing_position,
            current_joint_positions=manipulator.get_joint_positions(),
            endEffectorOffset=offset,
            endEffectorOrientation=endEffectorOrientation,
        )
        manipulator.apply_action(actions)

    def _motionPolicyState(self, sim, obj, manipulator, robot_name):
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
            self._syncMotionPolicyBase(manipulator, rmp_flow)
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

    def _syncMotionPolicyBase(self, manipulator, rmp_flow):
        position, orientation = manipulator.get_world_pose()
        rmp_flow.set_robot_base_pose(
            robot_position=position,
            robot_orientation=orientation,
        )

    def _manipulator(self, sim, obj):
        wrapper = sim.world.scene.get_object(obj.name)
        if not getattr(obj, "_core_manipulator_ready", False):
            profile = obj.manipulatorProfile
            if profile.gripperStyle == "robotiq_2f85":
                arm_dof_indices = self._coreDofIndices(wrapper, profile.armDofNames)
                wrapper.set_joint_positions(
                    profile.defaultArmPose,
                    joint_indices=np.asarray(arm_dof_indices, dtype=np.int32),
                )
            wrapper.gripper.set_joint_positions(wrapper.gripper.joint_opened_positions)
            obj._core_manipulator_ready = True
        return wrapper

    def moveManipulatorEndEffector(self, sim, obj, position, orientation=None):
        profile = obj.manipulatorProfile
        manipulator = self._manipulator(sim, obj)
        state = self._motionPolicyState(sim, obj, manipulator, profile.rmpflowPolicyName)
        self._syncMotionPolicyBase(manipulator, state["rmp_flow"])
        if orientation is None:
            orientation = profile.downwardOrientation
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        if profile.rmpflowUsesTcpOffset:
            target_position = self._tcpToControlPosition(
                position, orientation, profile.tcpOffset
            )
        else:
            target_position = _positionArray(position)
        action = state["controller"].forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=orientation,
        )
        manipulator.apply_action(action)

    def setManipulatorGripper(self, sim, obj, opened):
        profile = obj.manipulatorProfile
        manipulator = self._manipulator(sim, obj)
        if profile.gripperControlMode == "velocity":
            velocity = (
                profile.gripperOpenVelocity if opened else profile.gripperCloseVelocity
            )
            indices = self._coreDofIndices(manipulator, profile.gripperDofNames)
            manipulator.get_articulation_controller().switch_dof_control_mode(
                dof_index=indices[0],
                mode="velocity",
            )
            from isaacsim.core.utils.types import ArticulationAction

            action = ArticulationAction(
                joint_velocities=np.array([velocity], dtype=float),
                joint_indices=np.asarray(indices, dtype=np.int32),
            )
            manipulator.apply_action(action)
            return
        action = manipulator.gripper.forward(action="open" if opened else "close")
        manipulator.apply_action(action)

    def setManipulatorArmJointPositions(self, sim, obj, joint_positions):
        profile = obj.manipulatorProfile
        manipulator = self._manipulator(sim, obj)
        joints = np.asarray(joint_positions, dtype=float).reshape(-1)
        arm_dof_indices = self._coreDofIndices(manipulator, profile.armDofNames)
        if len(joints) > len(arm_dof_indices):
            raise RuntimeError(
                f"Arm joint target has more than {len(arm_dof_indices)} positions"
            )
        targets = [None] * manipulator.num_dof
        for index, value in zip(arm_dof_indices, joints):
            targets[index] = value
        from isaacsim.core.utils.types import ArticulationAction

        manipulator.apply_action(ArticulationAction(joint_positions=targets))

    def holdManipulatorPosition(self, sim, obj):
        profile = obj.manipulatorProfile
        manipulator = self._manipulator(sim, obj)
        arm_dof_indices = self._coreDofIndices(manipulator, profile.armDofNames)
        current = np.asarray(manipulator.get_joint_positions(), dtype=float)
        targets = [None] * manipulator.num_dof
        for index in arm_dof_indices:
            targets[index] = current[index]
        from isaacsim.core.utils.types import ArticulationAction

        manipulator.apply_action(ArticulationAction(joint_positions=targets))

    def getManipulatorEndEffectorPose(self, sim, obj):
        profile = obj.manipulatorProfile
        manipulator = self._manipulator(sim, obj)
        state = self._motionPolicyState(sim, obj, manipulator, profile.rmpflowPolicyName)
        self._syncMotionPolicyBase(manipulator, state["rmp_flow"])
        active_joints = state["policy"].get_active_joints_subset().get_joint_positions()
        position, orientation = state["rmp_flow"].get_end_effector_pose(active_joints)
        position = _asArray(position).reshape(-1)[:3]
        orientation = _asArray(orientation)
        if orientation.shape == (3, 3):
            from isaacsim.core.utils.rotations import rot_matrix_to_quat

            orientation = rot_matrix_to_quat(orientation)
        orientation = orientation.reshape(-1)[:4]
        if profile.rmpflowUsesTcpOffset:
            position = self._controlToTcpPosition(
                position, orientation, profile.tcpOffset
            )
        return position, orientation

    def getManipulatorGripperPositions(self, sim, obj):
        manipulator = self._manipulator(sim, obj)
        positions = np.asarray(manipulator.gripper.get_joint_positions(), dtype=float)
        return positions.reshape(-1)

    def manipulatorGripperTargetPositions(self, profile, opened):
        if opened:
            return profile.openGripperPositions.copy()
        return profile.closedGripperPositions.copy()

    def _coreDofIndices(self, articulation, names):
        dof_names = list(articulation.dof_names)
        missing = [name for name in names if name not in dof_names]
        if missing:
            raise RuntimeError(f"{articulation.name} is missing required DOFs: {missing}")
        return [dof_names.index(name) for name in names]

    def _tcpToControlPosition(self, tcp_position, orientation, tcp_offset):
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return _positionArray(tcp_position) - self.rotateVectorByWxyzQuat(
            orientation, tcp_offset
        )

    def _controlToTcpPosition(self, control_position, orientation, tcp_offset):
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        control_position = np.asarray(control_position, dtype=float).reshape(-1)[:3]
        return control_position + self.rotateVectorByWxyzQuat(orientation, tcp_offset)

    def getPhysicsProperties(self, world, obj):
        isaac_obj = world.scene.get_object(obj.name)
        position, orientation = isaac_obj.get_world_pose()
        x, y, z = position
        yaw, pitch, roll = self.isaacQuatToScenicEulerAngles(orientation)
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

    def _toCoreArticulationAction(self, action):
        from isaacsim.core.utils.types import ArticulationAction

        return ArticulationAction(**action)
