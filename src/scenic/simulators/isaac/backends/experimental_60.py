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


def _differential_inverse_kinematics(
    jacobian_end_effector,
    current_position,
    current_orientation,
    goal_position,
    goal_orientation=None,
    damping=0.05,
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
        transpose
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
        if world.app is not None:
            world.app.update()

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

    def _franka_end_effector_pose(self, handle):
        position, orientation = handle.metadata["end_effector"].get_world_poses()
        return position.numpy(), orientation.numpy()

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
        return [dof_names.index(name) for name in names]
