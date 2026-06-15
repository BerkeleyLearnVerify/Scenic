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
    """Isaac Sim 6.0.0 backend implemented with Core Experimental APIs."""

    name = "experimental_60"

    def create_world(self, timestep):
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
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.enable_extension(name)

    def initialize_physics(self, world, objects):
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.setup_simulation(dt=world.timestep)
        if world.app is not None:
            world.app.update()

    def play_world(self, world):
        import omni.timeline

        omni.timeline.get_timeline_interface().play()
        if world.app is not None:
            world.app.update()

    def step_world(self, world):
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.rendering_manager import RenderingManager

        SimulationManager.step(steps=1)
        RenderingManager.render()

        if world.app is not None:
            world.app.update()

        world.simulation_time = SimulationManager.get_simulation_time()

    def stop_and_clear_world(self, world):
        import isaacsim.core.experimental.utils.stage as stage_utils
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        if world.app is not None:
            world.app.update()

        for name, wrapper in list(world.objects.items()):
            prim_path = getattr(wrapper, "prim_path", f"/World/{name}")
            try:
                stage_utils.delete_prim(prim_path)
            except Exception as exc:
                pass
        world.objects.clear()
        if world.app is not None:
            world.app.update()

    def run_coroutine(self, coro):
        return self._simulation_app.run_coroutine(coro)

    def add_object(self, world, obj, *, scenic_obj=None):
        world.objects[scenic_obj.name] = obj

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
        asset_prim_path = f"{prim_path}/asset"

        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )

        stage_utils.define_prim(prim_path, "Xform")

        stage_utils.add_reference_to_stage(usd_path=usd_path, path=asset_prim_path)

        scenic_position = scenic_utils.vectorToArray(obj.position)
        orientation = self.scenic_to_isaac_orientation(obj.orientation)

        # Geometry is under /World/ObjectName/asset.
        geometry_paths = self._geometry_paths_under(prim_path)
        self._apply_collisions_to_geometry(geometry_paths)

        # Compute scale from Scenic dimensions to native USD dimensions.
        # This should compute the bbox of the full parent, including the asset child.
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
            self.disable_rigid_body(prim_path)

        if obj.color:
            self.apply_visual_material(wrapper, obj, geometry_paths=geometry_paths)

        return wrapper

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

        if getattr(obj, "wheel_controller", None) in {"differential", "holonomic", "ackermann"}:
            return self.create_wheeled_robot(obj)

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
            orientations=self.scenic_to_isaac_orientation(
                obj.orientation, initial_rotation=obj.initial_rotation
            ),
            reset_xform_op_properties=True,
        )
        if obj.control:
            obj.controller = obj.control
        if obj.color:
            self.apply_visual_material(wrapper, obj, geometry_paths=self._geometry_paths_under(prim_path))
        return wrapper
    
    def create_wheeled_robot(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.robot.experimental.wheeled_robots.robots import (
            WheeledRobot,
            HolonomicRobotUsdSetup,
        )
        from isaacsim.robot.experimental.wheeled_robots.controllers import (
            DifferentialController,
            HolonomicController,
            AckermannController,
        )

        prim_path = f"/World/{obj.name}"
        usd_path = (
            self.asset_path(obj.isaac_asset_path)
            if obj.isaac_asset_path
            else os.path.abspath(obj.usd_path)
        )

        wrapper = WheeledRobot(
            paths=prim_path,
            wheel_dof_names=obj.wheel_dof_names,
            usd_path=usd_path,
            positions=scenic_utils.vectorToArray(obj.position),
            orientations=self.scenic_to_isaac_orientation(obj.orientation, initial_rotation=obj.initial_rotation)
        )

        obj.wheel_dof_indices = wrapper.get_dof_indices(obj.wheel_dof_names)


        if obj.wheel_controller == "differential":
            obj.controller = DifferentialController(
                wheel_radius=obj.wheel_radius,
                wheel_base=obj.wheel_base,
            )
        elif obj.wheel_controller == "holonomic":
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
            obj.steering_dof_indices = wrapper.get_dof_indices(steering_dof_names)

            # If the user only exposes obj.wheel_radius in Scenic, use it for both front/back.
            front_wheel_radius = getattr(obj, "front_wheel_radius", obj.wheel_radius)
            back_wheel_radius = getattr(obj, "back_wheel_radius", obj.wheel_radius)

            obj.controller = AckermannController(
                wheel_base=obj.wheel_base,
                track_width=obj.track_width,
                front_wheel_radius=front_wheel_radius,
                back_wheel_radius=back_wheel_radius,
            )
        else:
            obj.controller = obj.control

        if obj.color:
            self.apply_visual_material(wrapper, obj)
        
        return wrapper

    def apply_robot_control(self, sim, obj, command):
        wrapper = sim.world.get_object(obj.name)
        if obj.controller is None:
            return
        if getattr(obj, "wheel_controller", None) in {"differential", "holonomic", "ackermann"}:
            self.apply_wheeled_control(sim, obj, command)
            return

        action = obj.controller(command)
        self._apply_articulation_action(wrapper, action)

    def apply_wheeled_control(self, sim, obj, command):
        wrapper = sim.world.get_object(obj.name)

        if obj.controller is None:
            return

        wheel_controller = obj.wheel_controller

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
                dof_indices=obj.steering_dof_indices,
            )
            # obj.wheel_dof_names should be ordered as: [front_left, front_right, rear_left, rear_right]
            wrapper.apply_wheel_actions(wheel_velocities)
            return

        # If the user supplied a custom controller that returns an existing ArticulationAction.
        action = obj.controller(command)
        self._apply_articulation_action(wrapper, action)

    def create_franka_panda(self, obj):
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation, RigidPrim

        prim_path = f"/World/{obj.name}"
        root_position = self._franka_root_position(obj)
        root_orientation = self.scenic_to_isaac_orientation(
            obj.orientation,
            initial_rotation=obj.initial_rotation,
        )

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

        obj._franka_metadata = {
            "prim_path": prim_path,
            "end_effector": end_effector,
            "end_effector_link_index": end_effector_link_index,
            "arm_dof_indices": list(range(7)),
            "arm_dof_count": 7,
            "gripper_dof_indices": self._dof_indices(
                wrapper,
                ["panda_finger_joint1", "panda_finger_joint2"],
            ),
            "default_dof_positions": default_dof_positions,
            "open_gripper_positions": np.array([0.05, 0.05], dtype=float),
            "closed_gripper_positions": np.array([0.01, 0.01], dtype=float),
            # Isaac quaternion convention: [w, x, y, z].
            "downward_orientation": np.array([0.0, 1.0, 0.0, 0.0], dtype=float),
        }

        obj._franka_pick_place_state = None

        if obj.color:
            self.apply_visual_material(
                wrapper,
                obj,
                geometry_paths=self._geometry_paths_under(prim_path),
            )

        return wrapper

    def _franka_root_position(self, obj):
        position = scenic_utils.vectorToArray(obj.position)
        position[2] -= obj.height / 2
        return position
    
    def move_franka_pick_place(
        self,
        sim,
        obj,
        target_object,
        goal_position,
        end_effector_offset=None,
        end_effector_orientation=None,
    ):
        franka = sim.world.get_object(obj.name)

        state = getattr(obj, "_franka_pick_place_state", None)
        if state is None:
            state = FrankaPickPlaceState()
            obj._franka_pick_place_state = state
            self._reset_franka(obj, franka)

        if state.done:
            return

        if end_effector_orientation is None:
            end_effector_orientation = obj.end_effector_orientation

        if end_effector_offset is None:
            end_effector_offset = obj.end_effector_offset

        end_effector_offset = np.asarray(end_effector_offset, dtype=float)

        self._move_franka_pick_place_helper(
            franka,
            obj,
            state,
            sim,
            target_object,
            goal_position,
            end_effector_offset,
            end_effector_orientation,
        )

    def _move_franka_pick_place_helper(
        self,
        franka,
        obj,
        state,
        sim,
        target_object,
        goal_position,
        end_effector_offset,
        end_effector_orientation=None,
    ):
        metadata = obj._franka_metadata

        if state.pick_position is None:
            target_wrapper = sim.world.get_object(target_object.name)
            state.pick_position = target_wrapper.get_world_poses()[0].numpy()[0].copy()

        if state.place_position is None:
            state.place_position = scenic_utils.vectorToArray(goal_position).copy()

        cube_position = state.pick_position
        place_position = state.place_position

        current_position, current_orientation = self._franka_end_effector_pose(obj)

        if end_effector_orientation is not None:
            state.end_effector_orientation = np.asarray(
                end_effector_orientation,
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
            self._move_franka_end_effector(
                franka,
                obj,
                current_position=current_position,
                current_orientation=current_orientation,
                goal_position=np.asarray([target + end_effector_offset], dtype=float),
                goal_orientation=np.asarray([orientation], dtype=float),
            )

        self._set_franka_gripper(franka, obj, gripper_state)

        state.stage_steps += 1
        if state.stage_steps > steps:
            print(
                f"Franka stage={state.stage}, steps={state.stage_steps}, "
                f"target={target}, gripper={gripper_state}",
                flush=True,
            )

            state.stage += 1
            state.stage_steps = 0

            if state.stage >= len(phases):
                state.done = True

    def _dof_indices(self, articulation, names):
        dof_names = list(articulation.dof_names)
        return [dof_names.index(name) for name in names]

    def _reset_franka(self, obj, franka):
        metadata = obj._franka_metadata

        franka.reset_to_default_state()
        franka.set_dof_position_targets(metadata["default_dof_positions"])
        self._set_franka_gripper(franka, obj, "open")


    def _franka_end_effector_pose(self, obj):
        metadata = obj._franka_metadata
        position, orientation = metadata["end_effector"].get_world_poses()
        return position.numpy(), orientation.numpy()


    def _move_franka_end_effector(
        self,
        franka,
        obj,
        current_position,
        current_orientation,
        goal_position,
        goal_orientation=None,
    ):
        metadata = obj._franka_metadata

        arm_dof_indices = metadata["arm_dof_indices"]
        arm_dof_count = metadata["arm_dof_count"]

        current_dof_positions = franka.get_dof_positions().numpy()
        jacobian_matrices = franka.get_jacobian_matrices().numpy()

        jacobian_end_effector = jacobian_matrices[
            :,
            metadata["end_effector_link_index"] - 1,
            :,
            :arm_dof_count,
        ]

        delta_dof_positions = _differential_inverse_kinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=np.asarray(current_position, dtype=float).reshape(1, 3),
            current_orientation=np.asarray(current_orientation, dtype=float).reshape(1, 4),
            goal_position=np.asarray(goal_position, dtype=float).reshape(1, 3),
            goal_orientation=(
                None
                if goal_orientation is None
                else np.asarray(goal_orientation, dtype=float).reshape(1, 4)
            ),
        )

        if current_dof_positions.ndim == 1:
            dof_position_targets = (
                current_dof_positions[arm_dof_indices] + delta_dof_positions[0]
            )
        else:
            dof_position_targets = (
                current_dof_positions[:, arm_dof_indices] + delta_dof_positions
            )

        franka.set_dof_position_targets(
            dof_position_targets,
            dof_indices=arm_dof_indices,
        )


    def _set_franka_gripper(self, franka, obj, state):
        metadata = obj._franka_metadata

        if state == "open":
            positions = metadata["open_gripper_positions"]
        elif state == "closed":
            positions = metadata["closed_gripper_positions"]

        franka.set_dof_position_targets(
            positions,
            dof_indices=metadata["gripper_dof_indices"],
        )

    def create_ground_plane(self, obj):
        from isaacsim.core.experimental.objects import GroundPlane

        wrapper = GroundPlane(
            "/World/GroundPlane",
            sizes=max(obj.width, obj.length),
            positions=[0, 0, 0],
        )
        if obj.color:
            self.apply_visual_material(wrapper, obj)
        return wrapper

    def apply_articulation_action(self, sim, obj, action):
        wrapper = sim.world.get_object(obj.name)
        self._apply_articulation_action(wrapper, action)

    def articulation_dof_names(self, sim, obj):
        wrapper = sim.world.get_object(obj.name)
        return list(wrapper.dof_names)

    def get_object_pose(self, sim, obj):
        wrapper = sim.world.get_object(obj.name)
        position, orientation = wrapper.get_world_poses()
        return position.numpy()[0], orientation.numpy()[0]

    def set_object_pose(self, sim, obj, position, orientation=None):
        wrapper = sim.world.get_object(obj.name)
        position = np.array(position, dtype=float)
        if orientation is None:
            _, orientation = self.get_object_pose(sim, obj)
        orientation = np.array(orientation, dtype=float)
        wrapper.set_world_poses(positions=position, orientations=orientation)
        if hasattr(wrapper, "set_velocities"):
            wrapper.set_velocities(
                linear_velocities=np.zeros(3, dtype=float),
                angular_velocities=np.zeros(3, dtype=float),
            )

    def get_physics_properties(self, world, obj):
        wrapper = world.get_object(obj.name)
        position, orientation = wrapper.get_world_poses()
        position = position.numpy()[0]
        orientation = orientation.numpy()[0]
        yaw, pitch, roll = self.isaac_quat_to_scenic_euler_angles(orientation)
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

    def _apply_articulation_action(self, articulation, action):
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
