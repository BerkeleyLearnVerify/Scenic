import math
import os
import numpy as np

from scenic.simulators.isaac.backends.base import IsaacBackend
import scenic.simulators.isaac.utils as scenic_utils


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
                        isaac_obj.get_dof_index(name)
                        for name in obj.steering_dof_names
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
            obj.controller = self.create_controller(
                obj.control, f"{obj.name}_controller"
            )

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
