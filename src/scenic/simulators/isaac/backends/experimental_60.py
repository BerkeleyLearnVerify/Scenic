from dataclasses import dataclass, field
import math
import os

import numpy as np

from scenic.simulators.isaac.backends.base import IsaacBackend
import scenic.simulators.isaac.utils as scenic_utils


@dataclass
class ExperimentalWorld:
    app: object
    timestep: float
    render: bool = False
    objects: dict = field(default_factory=dict)
    simulation_time: float = 0.0

    def getObject(self, name):
        return self.objects[name]


@dataclass
class ManipulatorPickPlaceState:
    stage: int = 0
    stage_steps: int = 0
    done: bool = False
    endEffectorOrientation: object = None
    pick_position: object = None
    place_position: object = None


def _quatMul(a, b):
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


def _quatConjugate(q):
    return np.concatenate((q[:, :1], -q[:, 1:]), axis=-1)


def _positionArray(position):
    if hasattr(position, "x") and hasattr(position, "y") and hasattr(position, "z"):
        return np.array([position.x, position.y, position.z], dtype=float)
    return np.asarray(position, dtype=float).reshape(-1)[:3]


def _differentialInverseKinematics(
    jacobian_end_effector,
    current_position,
    current_orientation,
    goalPosition,
    goal_orientation=None,
    damping=0.05,
    scale=1.0,
):
    goal_orientation = (
        current_orientation if goal_orientation is None else goal_orientation
    )
    q = _quatMul(goal_orientation, _quatConjugate(current_orientation))
    error = np.expand_dims(
        np.concatenate(
            [goalPosition - current_position, q[:, 1:] * np.sign(q[:, [0]])],
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
    """Isaac Sim 6.0.0 backend implemented with Core Experimental APIs."""

    name = "experimental_60"

    def createWorld(self, timestep):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from pxr import UsdPhysics

        if stage_utils.get_current_stage() is None:
            stage_utils.create_new_stage(template="sunlight")

        stage = stage_utils.get_current_stage()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)

        if not stage.GetPrimAtPath("/World/physicsScene").IsValid():
            UsdPhysics.Scene.Define(stage, "/World/physicsScene")

        return ExperimentalWorld(app=self._simulation_app, timestep=timestep)

    def openEnvironmentStage(self, usd_path):
        import isaacsim.core.experimental.utils.stage as stage_utils

        if self._stageAlreadyOpen(stage_utils, usd_path):
            return True

        opened, stage = stage_utils.open_stage(usd_path)
        if not opened:
            return False
        stage.SetEditTarget(stage.GetSessionLayer())
        self._environment_usd_path = usd_path
        return True

    def _stageAlreadyOpen(self, stage_utils, usd_path):
        if getattr(self, "_environment_usd_path", None) != usd_path:
            return False
        stage = stage_utils.get_current_stage()
        if stage is None:
            return False
        stage.SetEditTarget(stage.GetSessionLayer())
        return True

    def enableExtension(self, name):
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.enable_extension(name)

    def initializePhysics(self, world, objects):
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.setup_simulation(dt=world.timestep)
        self._configureManipulatorPickObjectsForWorld(world, objects)
        if world.app is not None:
            world.app.update()

    def _configureManipulatorPickObjectsForWorld(self, world, objects):
        profile = next(
            (
                obj.manipulatorProfile
                for obj in objects
                if getattr(obj, "manipulatorProfile", None) is not None
                and obj.manipulatorProfile.gripperStyle == "robotiq_2f85"
            ),
            None,
        )
        if profile is None:
            return

        import isaacsim.core.experimental.utils.stage as stage_utils

        stage = stage_utils.get_current_stage()
        pick_object_paths = [
            obj._isaac_generic_prim_path
            for obj in objects
            if obj.physics and hasattr(obj, "_isaac_generic_prim_path")
        ]
        if pick_object_paths:
            self._configureRobotiqPickObjectContact(stage, pick_object_paths, profile)

    def playWorld(self, world):
        import omni.timeline

        omni.timeline.get_timeline_interface().play()
        if world.app is not None:
            world.app.update()

    def stepWorld(self, world):
        from isaacsim.core.rendering_manager import RenderingManager
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.step(steps=1)
        RenderingManager.render()

        if world.app is not None:
            world.app.update()

        world.simulation_time = SimulationManager.get_simulation_time()

    def stopAndClearWorld(self, world):
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        if world.app is not None:
            world.app.update()

        for name, wrapper in list(world.objects.items()):
            prim_path = getattr(wrapper, "primPath", f"/World/{name}")
            try:
                stage_utils.delete_prim(prim_path)
            except Exception as exc:
                pass
        world.objects.clear()
        if world.app is not None:
            world.app.update()

    def runCoroutine(self, coro):
        return self._simulation_app.run_coroutine(coro)

    def addObject(self, world, obj, *, scenic_obj=None):
        world.objects[scenic_obj.name] = obj

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
            backend_name=self.name,
            open_stage_func=self._openStageForConversion,
        )
        return mesh_path, info_path

    def _openStageForConversion(self, usd_path):
        import isaacsim.core.experimental.utils.stage as stage_utils

        opened, stage = stage_utils.open_stage(usd_path)
        return opened

    def createGenericObject(self, obj):
        from isaacsim.core.experimental.prims import RigidPrim, XformPrim
        import isaacsim.core.experimental.utils.stage as stage_utils

        prim_path = f"/World/{obj.name}"
        assetPrimPath = f"{prim_path}/asset"

        usd_path = (
            self.assetPath(obj.isaacAssetPath)
            if obj.isaacAssetPath
            else os.path.abspath(obj.usdPath)
        )

        stage_utils.define_prim(prim_path, "Xform")

        stage_utils.add_reference_to_stage(usd_path=usd_path, path=assetPrimPath)

        scenic_position = scenic_utils.vectorToArray(obj.position)
        orientation = self.scenicToIsaacOrientation(obj.orientation)

        # Geometry is under /World/ObjectName/asset.
        geometry_paths = self._geometryPathsUnder(prim_path)
        self._applyCollisionsToGeometry(geometry_paths)

        # Compute scale from Scenic dimensions to native USD dimensions.
        # This should compute the bbox of the full parent, including the asset child.
        root_position, local_scale, native_size, native_center = (
            self.computeUsdScaleAndRootPosition(
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
                scales=local_scale,
                reset_xform_op_properties=True,
            )

            if obj.mass is not None:
                wrapper.set_masses(np.asarray([obj.mass], dtype=np.float32))

            if obj.density is not None:
                wrapper.set_densities(np.asarray([obj.density], dtype=np.float32))

            velocity = scenic_utils.vectorToArray(obj.velocity)
            wrapper.set_velocities(linear_velocities=velocity)

        else:
            wrapper = XformPrim(
                prim_path,
                positions=root_position,
                orientations=orientation,
                scales=local_scale,
                reset_xform_op_properties=True,
            )
            self.disableRigidBody(prim_path)

        if obj.color:
            self.applyVisualMaterial(wrapper, obj, geometry_paths=geometry_paths)

        obj._isaac_generic_prim_path = prim_path
        return wrapper

    def _geometryPathsUnder(self, prim_path):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from pxr import Usd, UsdGeom

        stage = stage_utils.get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        paths = []
        for descendant in Usd.PrimRange(prim):
            if descendant.IsA(UsdGeom.Gprim):
                paths.append(str(descendant.GetPath()))
        return paths

    def _applyCollisionsToGeometry(self, geometry_paths):
        if not geometry_paths:
            return
        from isaacsim.core.experimental.prims import GeomPrim

        geom = GeomPrim(geometry_paths, apply_collision_apis=True)
        geom.set_collision_approximations(["convexDecomposition"])

    def disableRigidBody(self, prim_path):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from pxr import Usd, UsdPhysics

        prim = stage_utils.get_current_stage().GetPrimAtPath(prim_path)
        for descendant in Usd.PrimRange(prim):
            if descendant.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_api = UsdPhysics.RigidBodyAPI(descendant)
                rigid_body_api.CreateRigidBodyEnabledAttr(False)

    def applyVisualMaterial(self, wrapper, obj, geometry_paths=None):
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

    def createRobot(self, obj):
        from isaacsim.core.experimental.prims import Articulation
        import isaacsim.core.experimental.utils.stage as stage_utils

        if getattr(obj, "manipulatorProfile", None) is not None:
            return self.createManipulator(obj)

        if getattr(obj, "wheelController", None) in {
            "differential",
            "holonomic",
            "ackermann",
        }:
            return self.createWheeledRobot(obj)

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.assetPath(obj.isaacAssetPath)
            if obj.isaacAssetPath
            else os.path.abspath(obj.usdPath)
        )
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=prim_path)
        wrapper = Articulation(
            prim_path,
            positions=scenic_utils.vectorToArray(obj.position),
            orientations=self.scenicToIsaacOrientation(
                obj.orientation, initial_rotation=obj.initialRotation
            ),
            reset_xform_op_properties=True,
        )
        if obj.control:
            obj.controller = obj.control
        if obj.color:
            self.applyVisualMaterial(
                wrapper, obj, geometry_paths=self._geometryPathsUnder(prim_path)
            )
        return wrapper

    def createWheeledRobot(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.robot.experimental.wheeled_robots.controllers import (
            AckermannController,
            DifferentialController,
            HolonomicController,
        )
        from isaacsim.robot.experimental.wheeled_robots.robots import (
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
            paths=prim_path,
            wheel_dof_names=obj.wheelDofNames,
            usd_path=usd_path,
            positions=scenic_utils.vectorToArray(obj.position),
            orientations=self.scenicToIsaacOrientation(
                obj.orientation, initial_rotation=obj.initialRotation
            ),
        )

        obj.wheelDofIndices = wrapper.get_dof_indices(obj.wheelDofNames)

        if obj.wheelController == "differential":
            obj.controller = DifferentialController(
                wheel_radius=obj.wheelRadius,
                wheel_base=obj.wheelBase,
            )
        elif obj.wheelController == "holonomic":
            holonomic_setup = HolonomicRobotUsdSetup(
                robot_prim_path=prim_path,
                com_prim_path=f"/World/{obj.name}/base_link/control_offset",
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
            obj.steeringDofIndices = wrapper.get_dof_indices(steering_dof_names)

            # If the user only exposes obj.wheelRadius in Scenic, use it for both front/back.
            front_wheel_radius = getattr(obj, "frontWheelRadius", obj.wheelRadius)
            back_wheel_radius = getattr(obj, "backWheelRadius", obj.wheelRadius)

            obj.controller = AckermannController(
                wheel_base=obj.wheelBase,
                track_width=obj.trackWidth,
                front_wheel_radius=front_wheel_radius,
                back_wheel_radius=back_wheel_radius,
            )
        else:
            obj.controller = obj.control

        if obj.color:
            self.applyVisualMaterial(wrapper, obj)

        return wrapper

    def applyRobotControl(self, sim, obj, command):
        wrapper = sim.world.getObject(obj.name)
        if obj.controller is None:
            return
        if getattr(obj, "wheelController", None) in {
            "differential",
            "holonomic",
            "ackermann",
        }:
            self.applyWheeledControl(sim, obj, command)
            return

        action = obj.controller(command)
        self._applyArticulationAction(wrapper, action)

    def applyWheeledControl(self, sim, obj, command):
        wrapper = sim.world.getObject(obj.name)

        if obj.controller is None:
            return

        wheel_controller = obj.wheelController

        if wheel_controller in {"differential", "holonomic"}:
            # Differential command: [linear_speed, angular_speed]
            # Holonomic command: [forward_speed, lateral_speed, yaw_speed]
            wrapper.apply_wheel_actions(obj.controller.forward(command))
            return
        elif wheel_controller == "ackermann":
            # Ackermann command: [steering_angle, steering_angle_velocity, speed, acceleration, dt]
            steering_positions, wheel_velocities = obj.controller.forward(command)

            wrapper.set_dof_position_targets(
                steering_positions,
                dof_indices=obj.steeringDofIndices,
            )
            # obj.wheelDofNames should be ordered as: [front_left, front_right, rear_left, rear_right]
            wrapper.apply_wheel_actions(wheel_velocities)
            return

        # If the user supplied a custom controller that returns an existing ArticulationAction.
        action = obj.controller(command)
        self._applyArticulationAction(wrapper, action)

    def createManipulator(self, obj):
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        import isaacsim.core.experimental.utils.stage as stage_utils

        profile = obj.manipulatorProfile
        prim_path = f"/World/{obj.name}"
        root_position = self._manipulatorRootPosition(obj)
        root_orientation = self.scenicToIsaacOrientation(
            obj.orientation,
            initial_rotation=obj.initialRotation,
        )

        robot_prim = stage_utils.add_reference_to_stage(
            usd_path=self.kitUsdPath(profile.usdPath),
            path=prim_path,
        )
        for variant_name, selection in profile.usdVariants:
            self._setRequiredVariant(robot_prim, variant_name, selection)

        stage = stage_utils.get_current_stage()
        self._requireStagePrim(stage, f"{prim_path}/{profile.endEffectorPrim}")
        if profile.gripperStyle == "robotiq_2f85":
            self._configureRobotiqGripperAttachment(stage, prim_path, profile)
            self._configureRobotiqDefaultJointPose(stage, prim_path, profile)
            self._configureRobotiqClosedLoopGripper(stage, prim_path, profile)
            self._configureRobotiqGripperDrive(stage, prim_path, profile)
            self._configureRobotiqGripperContact(stage, prim_path, profile)

        wrapper = Articulation(
            prim_path,
            positions=root_position,
            orientations=root_orientation,
            reset_xform_op_properties=True,
        )
        arm_dof_indices = self._dofIndices(wrapper, list(profile.armDofNames))
        if getattr(obj, "armMaxVelocities", None) is not None:
            wrapper.set_dof_max_velocities(
                obj.armMaxVelocities,
                dof_indices=arm_dof_indices,
            )
        gripper_dof_indices = self._dofIndices(wrapper, list(profile.gripperDofNames))
        default_dof_positions = np.zeros(len(wrapper.dof_names), dtype=float)
        for value, dof_index in zip(profile.defaultArmPose, arm_dof_indices):
            default_dof_positions[dof_index] = value
        for value, dof_index in zip(profile.openGripperPositions, gripper_dof_indices):
            default_dof_positions[dof_index] = value
        wrapper.set_default_state(dof_positions=default_dof_positions)

        end_effector = RigidPrim(f"{prim_path}/{profile.endEffectorPrim}")
        end_effector_link_index = self._linkIndex(wrapper, profile.controlLinkName)
        metadata = {
            "prim_path": prim_path,
            "end_effector": end_effector,
            "end_effector_link_index": end_effector_link_index,
            "arm_dof_indices": arm_dof_indices,
            "gripper_dof_indices": gripper_dof_indices,
            "default_dof_positions": default_dof_positions,
            "open_gripper_positions": profile.openGripperPositions.copy(),
            "closed_gripper_positions": profile.closedGripperPositions.copy(),
            "downward_orientation": profile.downwardOrientation.copy(),
            "tcp_offset": profile.tcpOffset.copy(),
        }
        obj._manipulator_metadata = metadata
        if profile.supportsPickPlace:
            obj._manipulator_pick_place_state = None

        if obj.color:
            self.applyVisualMaterial(
                wrapper,
                obj,
                geometry_paths=self._geometryPathsUnder(prim_path),
            )
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
        if stage is None:
            raise RuntimeError("Required Isaac USD stage is missing")
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

    def _configureRobotiqGripperContact(self, stage, prim_path, profile):
        from omni.physx.scripts import physicsUtils
        from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

        root = self._requireStagePrim(stage, f"{prim_path}/{profile.gripperPrim}")
        material = UsdShade.Material.Define(stage, profile.gripperContactMaterialPath)
        material_prim = material.GetPrim()
        material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
        material_api.CreateStaticFrictionAttr().Set(float(profile.gripperStaticFriction))
        material_api.CreateDynamicFrictionAttr().Set(
            float(profile.gripperDynamicFriction)
        )
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
                raise RuntimeError(
                    f"Could not bind gripper contact material to instance proxy: {prim.GetPath()}"
                )
            physicsUtils.add_physics_material_to_prim(
                stage, prim, profile.gripperContactMaterialPath
            )
            collision_api = (
                UsdPhysics.CollisionAPI(prim)
                if prim.HasAPI(UsdPhysics.CollisionAPI)
                else UsdPhysics.CollisionAPI.Apply(prim)
            )
            collision_api.CreateCollisionEnabledAttr().Set(True)
            physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_collision_api.CreateContactOffsetAttr().Set(
                float(profile.contactOffset)
            )
            physx_collision_api.CreateRestOffsetAttr().Set(float(profile.restOffset))
            bound.append(str(prim.GetPath()))
        if not bound:
            raise RuntimeError(
                f"No collision geometry found for Robotiq gripper under {root.GetPath()}"
            )

    def _configureRobotiqPickObjectContact(self, stage, prim_paths, profile):
        from omni.physx.scripts import physicsUtils
        from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

        material = UsdShade.Material.Define(stage, profile.objectContactMaterialPath)
        material_prim = material.GetPrim()
        material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
        material_api.CreateStaticFrictionAttr().Set(float(profile.objectStaticFriction))
        material_api.CreateDynamicFrictionAttr().Set(float(profile.objectDynamicFriction))
        material_api.CreateRestitutionAttr().Set(0.0)

        physx_material_api = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
        physx_material_api.CreateFrictionCombineModeAttr().Set("max")
        physx_material_api.CreateRestitutionCombineModeAttr().Set("min")

        for prim_path in prim_paths:
            root = self._requireStagePrim(stage, prim_path)
            rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
            rigid_api.CreateEnableCCDAttr().Set(True)
            rigid_api.CreateSleepThresholdAttr().Set(0.0)

            mass_api = (
                UsdPhysics.MassAPI(root)
                if root.HasAPI(UsdPhysics.MassAPI)
                else UsdPhysics.MassAPI.Apply(root)
            )
            mass_api.CreateMassAttr().Set(float(profile.pickObjectMassKg))

            collision_prims = [
                prim
                for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                if any("Collision" in schema for schema in prim.GetAppliedSchemas())
            ]
            if not collision_prims:
                raise RuntimeError(
                    f"No collision geometry found for pick object: {prim_path}"
                )
            for prim in collision_prims:
                if prim.IsInstanceProxy():
                    raise RuntimeError(
                        f"Could not bind pick object material to instance proxy: {prim.GetPath()}"
                    )
                physicsUtils.add_physics_material_to_prim(
                    stage, prim, profile.objectContactMaterialPath
                )
                collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                collision_api.CreateContactOffsetAttr().Set(float(profile.contactOffset))
                collision_api.CreateRestOffsetAttr().Set(float(profile.restOffset))

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

    def _linkIndex(self, articulation, name):
        indices = articulation.get_link_indices(name).list()
        if len(indices) != 1:
            raise RuntimeError(f"Expected one link named {name!r}, found {len(indices)}")
        return indices[0]

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
        wrapper = sim.world.getObject(obj.name)

        state = getattr(obj, "_manipulator_pick_place_state", None)
        if state is None:
            state = ManipulatorPickPlaceState()
            obj._manipulator_pick_place_state = state
            self._resetManipulator(obj, wrapper)

        if state.done:
            return

        if endEffectorOrientation is None:
            endEffectorOrientation = obj.end_effector_orientation

        if endEffectorOffset is None:
            endEffectorOffset = obj.end_effector_offset

        endEffectorOffset = np.asarray(endEffectorOffset, dtype=float)

        self._moveManipulatorPickPlaceHelper(
            wrapper,
            obj,
            state,
            sim,
            targetObject,
            goalPosition,
            endEffectorOffset,
            endEffectorOrientation,
        )

    def _moveManipulatorPickPlaceHelper(
        self,
        wrapper,
        obj,
        state,
        sim,
        targetObject,
        goalPosition,
        endEffectorOffset,
        endEffectorOrientation=None,
    ):
        metadata = obj._manipulator_metadata

        if state.pick_position is None:
            target_wrapper = sim.world.getObject(targetObject.name)
            state.pick_position = target_wrapper.get_world_poses()[0].numpy()[0].copy()

        if state.place_position is None:
            state.place_position = scenic_utils.vectorToArray(goalPosition).copy()

        cube_position = state.pick_position
        place_position = state.place_position

        current_position, current_orientation = self._manipulatorEndEffectorPose(obj)

        if endEffectorOrientation is not None:
            state.end_effector_orientation = np.asarray(
                endEffectorOrientation,
                dtype=float,
            ).copy()
        elif state.end_effector_orientation is None:
            state.end_effector_orientation = metadata["downward_orientation"].copy()

        orientation = state.end_effector_orientation

        phases = [
            (cube_position + np.array([0.0, 0.0, 0.20]), "open", 120),
            (cube_position + np.array([0.0, 0.0, 0.10]), "open", 80),
            (None, "closed", 50),
            (cube_position + np.array([0.0, 0.0, 0.50]), "closed", 150),
            (place_position + np.array([0.0, 0.0, 0.50]), "closed", 180),
            (place_position + np.array([0.0, 0.0, 0.20]), "closed", 90),
            (None, "open", 20),
        ]

        target, gripper_state, steps = phases[state.stage]

        if target is not None:
            self._moveManipulatorEndEffector(
                wrapper,
                obj,
                current_position=current_position,
                current_orientation=current_orientation,
                goalPosition=np.asarray([target + endEffectorOffset], dtype=float),
                goal_orientation=np.asarray([orientation], dtype=float),
            )

        self._setManipulatorGripper(wrapper, obj, gripper_state)

        state.stage_steps += 1
        if state.stage_steps > steps:
            print(
                f"Pick-place stage={state.stage}, steps={state.stage_steps}, "
                f"target={target}, gripper={gripper_state}",
                flush=True,
            )

            state.stage += 1
            state.stage_steps = 0

            if state.stage >= len(phases):
                state.done = True

    def _dofIndices(self, articulation, names):
        dof_names = list(articulation.dof_names)
        missing = [name for name in names if name not in dof_names]
        if missing:
            raise RuntimeError(
                f"{articulation.paths[0]} is missing required DOFs: {missing}"
            )
        return [dof_names.index(name) for name in names]

    def _resetManipulator(self, obj, wrapper):
        metadata = obj._manipulator_metadata
        wrapper.reset_to_default_state()
        wrapper.set_dof_position_targets(metadata["default_dof_positions"])
        if obj.manipulatorProfile.gripperControlMode == "position":
            self._setManipulatorGripper(wrapper, obj, "open")

    def _ensureManipulatorControlReady(self, obj, wrapper):
        metadata = obj._manipulator_metadata
        if not metadata.get("primitive_control_ready", False):
            self._resetManipulator(obj, wrapper)
            metadata["primitive_control_ready"] = True

    def _manipulatorEndEffectorPose(self, obj):
        metadata = obj._manipulator_metadata
        position, orientation = metadata["end_effector"].get_world_poses()
        return position.numpy(), orientation.numpy()

    def _manipulatorTcpPosition(self, obj, control_position, orientation):
        tcp_offset = obj._manipulator_metadata["tcp_offset"]
        control_position = np.asarray(control_position, dtype=float).reshape(-1)[:3]
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return control_position + self.rotateVectorByWxyzQuat(orientation, tcp_offset)

    def _manipulatorControlPosition(self, obj, tcp_position, orientation):
        tcp_offset = obj._manipulator_metadata["tcp_offset"]
        tcp_position = _positionArray(tcp_position)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return tcp_position - self.rotateVectorByWxyzQuat(orientation, tcp_offset)

    def _moveManipulatorEndEffector(
        self,
        wrapper,
        obj,
        current_position,
        current_orientation,
        goalPosition,
        goal_orientation=None,
    ):
        profile = obj.manipulatorProfile
        metadata = obj._manipulator_metadata
        arm_dof_indices = metadata["arm_dof_indices"]

        current_dof_positions = wrapper.get_dof_positions().numpy()
        jacobian_matrices = wrapper.get_jacobian_matrices().numpy()

        jacobian_link_index = metadata["end_effector_link_index"] - 1
        if jacobian_link_index < 0:
            raise RuntimeError(
                "Manipulator control link index cannot be used for Jacobian"
            )
        jacobian_end_effector = np.take(
            jacobian_matrices[:, jacobian_link_index, :, :],
            arm_dof_indices,
            axis=-1,
        )

        delta_dof_positions = _differentialInverseKinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=np.asarray(current_position, dtype=float).reshape(1, 3),
            current_orientation=np.asarray(current_orientation, dtype=float).reshape(
                1, 4
            ),
            goalPosition=np.asarray(goalPosition, dtype=float).reshape(1, 3),
            goal_orientation=(
                None
                if goal_orientation is None
                else np.asarray(goal_orientation, dtype=float).reshape(1, 4)
            ),
            damping=profile.ikDamping,
            scale=profile.ikStepScale,
        )

        if current_dof_positions.ndim == 1:
            dof_position_targets = (
                current_dof_positions[arm_dof_indices] + delta_dof_positions[0]
            )
        else:
            dof_position_targets = (
                current_dof_positions[:, arm_dof_indices] + delta_dof_positions
            )

        wrapper.set_dof_position_targets(
            dof_position_targets,
            dof_indices=arm_dof_indices,
        )

    def _setManipulatorGripper(self, wrapper, obj, state):
        profile = obj.manipulatorProfile
        metadata = obj._manipulator_metadata
        indices = metadata["gripper_dof_indices"]

        if profile.gripperControlMode == "velocity":
            velocity = (
                profile.gripperOpenVelocity
                if state == "open"
                else profile.gripperCloseVelocity
            )
            wrapper.switch_dof_control_mode("velocity", dof_indices=indices)
            wrapper.set_dof_velocity_targets(
                np.full((1, len(indices)), float(velocity), dtype=float),
                dof_indices=indices,
            )
            return

        positions = (
            metadata["open_gripper_positions"]
            if state == "open"
            else metadata["closed_gripper_positions"]
        )
        wrapper.set_dof_position_targets(
            positions,
            dof_indices=indices,
        )

    def createGroundPlane(self, obj):
        from isaacsim.core.experimental.objects import GroundPlane

        wrapper = GroundPlane(
            "/World/GroundPlane",
            sizes=max(obj.width, obj.length),
            positions=[0, 0, 0],
        )
        if obj.color:
            self.applyVisualMaterial(wrapper, obj)
        return wrapper

    def applyArticulationAction(self, sim, obj, action):
        wrapper = sim.world.getObject(obj.name)
        self._applyArticulationAction(wrapper, action)

    def articulationDofNames(self, sim, obj):
        wrapper = sim.world.getObject(obj.name)
        return list(wrapper.dof_names)

    def getObjectPose(self, sim, obj):
        wrapper = sim.world.getObject(obj.name)
        position, orientation = wrapper.get_world_poses()
        return position.numpy()[0], orientation.numpy()[0]

    def setObjectPose(self, sim, obj, position, orientation=None):
        wrapper = sim.world.getObject(obj.name)
        position = np.array(position, dtype=float)
        if orientation is None:
            _, orientation = self.getObjectPose(sim, obj)
        orientation = np.array(orientation, dtype=float)
        wrapper.set_world_poses(positions=position, orientations=orientation)
        if hasattr(wrapper, "set_velocities"):
            wrapper.set_velocities(
                linear_velocities=np.zeros(3, dtype=float),
                angular_velocities=np.zeros(3, dtype=float),
            )

    def moveManipulatorEndEffector(self, sim, obj, position, orientation=None):
        wrapper = sim.world.getObject(obj.name)
        self._ensureManipulatorControlReady(obj, wrapper)
        current_position, current_orientation = self._manipulatorEndEffectorPose(obj)
        if orientation is None:
            orientation = obj._manipulator_metadata["downward_orientation"]
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        goalPosition = self._manipulatorControlPosition(obj, position, orientation)
        self._moveManipulatorEndEffector(
            wrapper,
            obj,
            current_position,
            current_orientation,
            np.array([goalPosition], dtype=float),
            np.array([orientation], dtype=float),
        )

    def setManipulatorGripper(self, sim, obj, opened):
        wrapper = sim.world.getObject(obj.name)
        self._ensureManipulatorControlReady(obj, wrapper)
        self._setManipulatorGripper(wrapper, obj, "open" if opened else "closed")

    def setManipulatorArmJointPositions(self, sim, obj, joint_positions):
        wrapper = sim.world.getObject(obj.name)
        self._ensureManipulatorControlReady(obj, wrapper)
        joint_positions = np.asarray(joint_positions, dtype=float).reshape(-1)
        arm_dof_indices = obj._manipulator_metadata["arm_dof_indices"]
        if len(joint_positions) > len(arm_dof_indices):
            raise RuntimeError(
                f"Arm joint target has more than {len(arm_dof_indices)} positions"
            )
        wrapper.set_dof_position_targets(
            joint_positions.tolist(),
            dof_indices=arm_dof_indices[: len(joint_positions)],
        )

    def holdManipulatorPosition(self, sim, obj):
        wrapper = sim.world.getObject(obj.name)
        self._ensureManipulatorControlReady(obj, wrapper)
        arm_dof_indices = obj._manipulator_metadata["arm_dof_indices"]
        current_dof_positions = wrapper.get_dof_positions().numpy()
        arm_targets = current_dof_positions[:, arm_dof_indices].reshape(-1).tolist()
        wrapper.set_dof_position_targets(
            arm_targets,
            dof_indices=arm_dof_indices,
        )

    def getManipulatorEndEffectorPose(self, sim, obj):
        position, orientation = self._manipulatorEndEffectorPose(obj)
        orientation = np.asarray(orientation, dtype=float).reshape(-1)[:4]
        return (
            self._manipulatorTcpPosition(obj, position, orientation),
            orientation,
        )

    def getManipulatorGripperPositions(self, sim, obj):
        wrapper = sim.world.getObject(obj.name)
        gripper_dof_indices = obj._manipulator_metadata["gripper_dof_indices"]
        dof_positions = wrapper.get_dof_positions().numpy()
        return dof_positions[:, gripper_dof_indices].reshape(-1)

    def manipulatorGripperTargetPositions(self, profile, opened):
        if opened:
            return profile.openGripperPositions.copy()
        return profile.closedGripperPositions.copy()

    def getPhysicsProperties(self, world, obj):
        wrapper = world.getObject(obj.name)
        position, orientation = wrapper.get_world_poses()
        position = position.numpy()[0]
        orientation = orientation.numpy()[0]
        yaw, pitch, roll = self.isaacQuatToScenicEulerAngles(orientation)
        linear_velocity, angular_velocity = wrapper.get_velocities()
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

    def _applyArticulationAction(self, articulation, action):
        dof_indices = action.get("joint_indices", action.get("dof_indices"))
        if "joint_positions" in action:
            articulation.set_dof_position_targets(
                action["joint_positions"],
                dof_indices=action.get("joint_position_indices", dof_indices),
            )
        if "joint_velocities" in action:
            articulation.set_dof_velocity_targets(
                action["joint_velocities"],
                dof_indices=action.get("joint_velocity_indices", dof_indices),
            )
        if "joint_efforts" in action:
            articulation.set_dof_efforts(
                action["joint_efforts"],
                dof_indices=action.get("joint_effort_indices", dof_indices),
            )
