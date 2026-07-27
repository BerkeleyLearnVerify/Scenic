from __future__ import annotations

import copy
import importlib
import math
import os
import tempfile
from typing import Any

from scenic.core.simulators import Simulation, SimulationCreationError, Simulator
from scenic.core.vectors import Vector
from scenic.simulators.isaac.backends import getBackend
from scenic.simulators.isaac.terrain_utils import buildScenicTerrainData

DEFAULT_EMPTY_ENV_CFG = "scenic.simulators.isaac.empty_env_cfg:ScenicEmptyEnvCfg"


class IsaacLabSimulator(Simulator):
    """Scenic simulator backend for Isaac Lab manager-based environments.

    Supported construction modes:
        1. env=<already-created Isaac Lab env>
        2. task="Isaac-Cartpole-v0" or another registered Isaac Lab task
        3. env_cfg=<ManagerBasedEnvCfg / ManagerBasedRLEnvCfg instance or class>
        4. env_cfg_entry_point="package.module:CfgClass"

    Use cases:
        IsaacLabSimulator(task="Isaac-...", num_envs=..., device=...)
    or:
        IsaacLabSimulator(env_cfg=MyScenicIsaacLabEnvCfg, num_envs=...)
    """

    def __init__(
        self,
        *,
        env: Any | None = None,
        task: str | None = None,
        env_cfg: Any | None = None,
        env_cfg_entry_point: str | None = None,
        env_cls: Any | None = None,
        timestep: float | None = 0.01,
        decimation: int | None = None,
        num_envs: int | None = None,
        env_spacing: float | None = None,
        terrainBorderWidth: float = 20.0,
        environmentUSDPath: str | os.PathLike | None = None,
        headless: bool = False,
        device: str | None = None,
        use_fabric: bool = True,
        render_mode: str | None = None,
        app_launcher_args: dict[str, Any] | None = None,
        debug_lifecycle: bool = True,
        **kwargs,
    ):
        super().__init__()

        self.env = env
        self.task = task
        self.env_cfg = env_cfg
        self.env_cfg_entry_point = env_cfg_entry_point
        self.env_cls = env_cls

        self.timestep = timestep
        self.decimation = decimation
        self.num_envs = num_envs
        self.env_spacing = env_spacing
        self.terrainBorderWidth = terrainBorderWidth
        self.environmentUSDPath = environmentUSDPath

        self.headless = headless
        self.device = device
        self.use_fabric = use_fabric
        self.render_mode = render_mode

        self.app_launcher_args = dict(app_launcher_args or {})
        self.app_launcher = None
        self.client = None
        self.backend = getBackend("lab")

        self.terrain_data = None

        self.debug_lifecycle = debug_lifecycle

        # If an env is already provided, assume the caller owns the app/env.
        if self.env is None:
            self._ensureApp()

        if (
            self.env is None
            and self.task is None
            and self.env_cfg is None
            and self.env_cfg_entry_point is None
        ):
            self.env_cfg_entry_point = DEFAULT_EMPTY_ENV_CFG

    def _ensureApp(self):
        """Launch Isaac Sim through Isaac Lab's AppLauncher."""
        if self.client is not None:
            return self.client

        self.client = self.backend.ensureApp(
            headless=self.headless,
            device=self.device,
            app_launcher_args=self.app_launcher_args,
        )
        self.app_launcher = self.backend.app_launcher
        return self.client

    def createSimulation(self, scene, **kwargs):
        timestep = self.timestep if kwargs.get("timestep") is None else kwargs["timestep"]
        kwargs.pop("timestep", None)

        return IsaacLabSimulation(
            scene,
            self,
            env=self.env,
            task=self.task,
            env_cfg=self.env_cfg,
            env_cfg_entry_point=self.env_cfg_entry_point,
            env_cls=self.env_cls,
            environmentUSDPath=self.environmentUSDPath,
            headless=self.headless,
            device=self.device,
            use_fabric=self.use_fabric,
            render_mode=self.render_mode,
            terrainBorderWidth=self.terrainBorderWidth,
            timestep=timestep,
            decimation=self.decimation,
            num_envs=self.num_envs,
            env_spacing=self.env_spacing,
            debug_lifecycle=self.debug_lifecycle,
            **kwargs,
        )

    def destroy(self):
        super().destroy()

        # If the user passed an existing env, do not close their app here.
        if self.env is not None:
            return

        if self.client is not None:
            self.backend.closeApp()
            self.client = None
            self.app_launcher = None


class IsaacLabSimulation(Simulation):
    """A Scenic Simulation backed by an Isaac Lab manager-based environment."""

    def __init__(
        self,
        scene,
        simulator: IsaacLabSimulator,
        *,
        env: Any | None = None,
        task: str | None = None,
        env_cfg: Any | None = None,
        env_cfg_entry_point: str | None = None,
        env_cls: Any | None = None,
        environmentUSDPath: str | os.PathLike | None = None,
        headless: bool = False,
        device: str | None = None,
        use_fabric: bool = True,
        render_mode: str | None = None,
        terrainBorderWidth: float = 20.0,
        timestep: float | None,
        decimation: int | None = None,
        num_envs: int | None = None,
        env_spacing: float | None = None,
        debug_lifecycle: bool = True,
        **kwargs,
    ):
        kwargs.setdefault("maxSteps", None)
        kwargs.setdefault("name", "IsaacLabSimulation")

        self.simulator = simulator

        self.env = env
        self._owns_env = env is None

        self.task = task
        self.raw_env_cfg = env_cfg
        self.env_cfg_entry_point = env_cfg_entry_point
        self.env_cls = env_cls

        self.environmentUSDPath = environmentUSDPath
        self.headless = headless
        self.device = device
        self.use_fabric = use_fabric
        self.render_mode = render_mode

        self.terrainBorderWidth = terrainBorderWidth
        self.timestep = timestep
        self.decimation = decimation
        self.num_envs = num_envs
        self.env_spacing = env_spacing

        self.tmpMeshDir = tempfile.mkdtemp()

        # Scenic objects are collected during Scenic's setup/create-object phase.
        # They are not spawned immediately.
        self.scenic_objects = []
        self.scenic_existing_objects = []
        self.terrains = []

        self.terrain_data = None
        self.env_cfg = None

        # Maps Scenic object names to Isaac Lab scene entity names.
        self._object_name_to_asset_name: dict[str, str] = {}

        # Most recent action/output data.
        self._pending_lab_action = None
        self._pending_robot_commands = {}
        self._last_step_output = None
        self._has_reset = False

        # Scenic robot bookkeeping.
        self._scenic_robot_asset_names = {}

        if self._owns_env:
            self.simulator._ensureApp()

        self.debug_lifecycle = debug_lifecycle
        self._step_count = 0

        self.backend = getBackend("lab")

        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        """Build the Isaac Lab env after Scenic has collected all objects."""
        super().setup()

        if self.env is None:
            try:
                self.env_cfg = self._buildEnvCfg()
                self.env = self._makeEnv(self.env_cfg)
            except Exception as exc:
                import traceback

                print(
                    "[SCENIC ISAAC LAB ERROR] Failed while creating Isaac Lab simulation:",
                    type(exc).__name__,
                    exc,
                    flush=True,
                )
                traceback.print_exc()
                raise

        self._resetEnvOnce()

    def _buildEnvCfg(self):
        """Create, validate, and patch an Isaac Lab manager-based env cfg."""
        cfg = self._materializeEnvCfg()
        self._validateManagerBasedCfg(cfg)
        self._applyStandardOverrides(cfg)
        self._applyScenicToEnvCfg(cfg)
        return cfg

    def _materializeEnvCfg(self):
        """Resolve env cfg from task, cfg object/class, or entry point."""

        if self.raw_env_cfg is not None:
            return self._instantiateCfg(self.raw_env_cfg)

        if self.env_cfg_entry_point is not None:
            return self._instantiateCfg(self.env_cfg_entry_point)

        if self.task is not None:
            import isaaclab_tasks
            from isaaclab_tasks.utils import parse_env_cfg

            return parse_env_cfg(
                self.task,
                device=self.device,
                num_envs=self.num_envs,
                use_fabric=self.use_fabric,
            )

        raise SimulationCreationError(
            "IsaacLabSimulation requires one of: env, task, env_cfg, or env_cfg_entry_point."
        )

    def _instantiateCfg(self, cfg_like):
        if isinstance(cfg_like, str):
            return self._instantiateCfg(self._loadEntryPoint(cfg_like))

        if isinstance(cfg_like, type):
            return cfg_like()

        if callable(cfg_like) and not hasattr(cfg_like, "scene"):
            return cfg_like()

        return copy.deepcopy(cfg_like)

    def _loadEntryPoint(self, entry_point: str):
        """Load 'package.module:ClassName'."""
        if ":" not in entry_point:
            raise SimulationCreationError(
                f"Invalid env_cfg_entry_point {entry_point!r}; expected 'module.submodule:ClassName'."
            )

        module_name, attr_name = entry_point.split(":", 1)
        try:
            module = importlib.import_module(module_name)
            return getattr(module, attr_name)
        except Exception as exc:
            raise SimulationCreationError(
                f"Could not load Isaac Lab env cfg entry point {entry_point!r}."
            ) from exc

    def _validateManagerBasedCfg(self, cfg):
        from isaaclab.envs import ManagerBasedEnvCfg, ManagerBasedRLEnvCfg

        if not isinstance(cfg, (ManagerBasedEnvCfg, ManagerBasedRLEnvCfg)):
            raise SimulationCreationError(
                "The Isaac Lab Scenic backend currently expects a manager-based config: "
                "ManagerBasedEnvCfg or ManagerBasedRLEnvCfg. "
                f"Got {type(cfg).__name__}. If you selected an Isaac Lab task, make sure it is "
                "not one of the '-Direct-' environments."
            )

    def _applyStandardOverrides(self, cfg):
        """Apply common simulator-level overrides to the env cfg."""
        if self.device is not None and hasattr(cfg, "sim"):
            cfg.sim.device = self.device

        if self.timestep is not None and hasattr(cfg, "sim"):
            cfg.sim.dt = self.timestep

        if self.decimation is not None and hasattr(cfg, "decimation"):
            cfg.decimation = self.decimation

        if (
            self.use_fabric is not None
            and hasattr(cfg, "sim")
            and hasattr(cfg.sim, "use_fabric")
        ):
            cfg.sim.use_fabric = self.use_fabric

        if self.num_envs is not None and hasattr(cfg, "scene"):
            cfg.scene.num_envs = self.num_envs

        if self.env_spacing is not None and hasattr(cfg, "scene"):
            cfg.scene.env_spacing = self.env_spacing

    def _applyScenicToEnvCfg(self, cfg):
        """Patch Isaac Lab cfg using objects sampled by Scenic.

        - If a Scenic object name matches an existing cfg.scene field, patch
            that existing Lab asset's initial pose.
        - If it does not match, create a new AssetBaseCfg or RigidObjectCfg
            from the Scenic object.
        - If a Scenic object provides obj.lab_asset_cfg, use that directly.
        - Terrain is collected and stored, but the actual TerrainImporterCfg
            integration is handled by a custom hook/config.
        """
        if not hasattr(cfg, "scene"):
            raise SimulationCreationError("Isaac Lab env cfg has no .scene field.")

        if self.terrains:
            self.terrain_data = buildScenicTerrainData(
                self.terrains,
                border_width=self.terrainBorderWidth,
            )
            self.simulator.terrain_data = self.terrain_data
            self._installScenicTerrainIntoCfg(cfg, self.terrain_data)

        if self.environmentUSDPath is not None:
            self._addEnvironmentUsdToCfg(cfg)
        else:
            for obj in self.scenic_existing_objects:
                self._patchOrRegisterScenicObject(cfg, obj, must_exist=True)

        for obj in self.scenic_objects:
            self._patchOrRegisterScenicObject(cfg, obj, must_exist=False)

    def _installScenicTerrainIntoCfg(self, cfg, terrain_data):
        """Install Scenic terrain into an Isaac Lab env config."""
        if not hasattr(cfg, "scene") or not hasattr(cfg.scene, "terrain"):
            raise SimulationCreationError(
                "This Isaac Lab env cfg has no cfg.scene.terrain. "
                "Scenic Terrain objects require a task/config with a TerrainImporterCfg, "
                "or you need to fall back to spawning the terrain mesh as a static collider."
            )

        from scenic.simulators.isaac.lab_env import configureEnvCfgForScenicTerrain

        configureEnvCfgForScenicTerrain(cfg, terrain_data)

    def _addEnvironmentUsdToCfg(self, cfg):
        """Add the environment USD under each Isaac Lab environment namespace."""
        setattr(
            cfg.scene,
            "scenic_environment",
            self.backend.makeEnvironmentCfg(self.environmentUSDPath),
        )

    def _patchOrRegisterScenicObject(self, cfg, obj, *, must_exist: bool):
        scenic_name = getattr(obj, "name", obj.__class__.__name__)
        asset_name = self.backend._safeAssetName(scenic_name)

        if hasattr(cfg.scene, asset_name):
            lab_asset_cfg = getattr(cfg.scene, asset_name)
            self.backend.patchAssetInitialPose(lab_asset_cfg, obj)
        elif must_exist:
            raise SimulationCreationError(
                f"Existing object {scenic_name!r} has no matching cfg.scene field {asset_name!r}."
            )
        else:
            lab_asset_cfg = self.backend.makeAssetCfg(
                obj,
                asset_name,
                num_envs=self.num_envs,
                tmp_mesh_dir=self.tmpMeshDir,
            )
            setattr(cfg.scene, asset_name, lab_asset_cfg)

        self._object_name_to_asset_name[scenic_name] = asset_name

        if getattr(obj, "blueprint", None) == "Robot":
            self._scenic_robot_asset_names[scenic_name] = asset_name

    def _makeEnv(self, cfg):
        """Construct the actual Isaac Lab environment."""
        if self.task is not None:
            import gymnasium as gym
            import isaaclab_tasks  # noqa: F401

            # gym.make uses the task's registered entry_point, usually:
            # "isaaclab.envs:ManagerBasedRLEnv"
            return gym.make(self.task, cfg=cfg, render_mode=self.render_mode)

        env_cls = self._resolveEnvCls(cfg)

        from isaaclab.envs import ManagerBasedRLEnv

        if issubclass(env_cls, ManagerBasedRLEnv):
            return env_cls(cfg=cfg, render_mode=self.render_mode)

        return env_cls(cfg=cfg)

    def _resolveEnvCls(self, cfg):
        """Pick ManagerBasedEnv or ManagerBasedRLEnv from the cfg type."""
        if self.env_cls is not None:
            if isinstance(self.env_cls, str):
                return self._loadEntryPoint(self.env_cls)
            return self.env_cls

        from isaaclab.envs import (
            ManagerBasedEnv,
            ManagerBasedEnvCfg,
            ManagerBasedRLEnv,
            ManagerBasedRLEnvCfg,
        )

        if isinstance(cfg, ManagerBasedRLEnvCfg):
            return ManagerBasedRLEnv
        if isinstance(cfg, ManagerBasedEnvCfg):
            return ManagerBasedEnv

        raise SimulationCreationError(
            f"Unsupported Isaac Lab cfg type: {type(cfg).__name__}"
        )

    def createObjectInSimulator(self, obj):
        """Collect Scenic objects instead of spawning them immediately."""
        blueprint = getattr(obj, "blueprint", None)

        if blueprint == "Terrain":
            self.terrains.append(obj)
            return None

        if blueprint == "ExistingIsaacSimObject":
            self.scenic_existing_objects.append(obj)
            return None

        # All objects are translated into env_cfg.scene.
        self.scenic_objects.append(obj)
        return None

    def executeActions(self, allActions):
        """Execute Scenic actions and prepare the Isaac Lab action tensor.

        There are two separate action paths:

        1. Isaac Lab task action tensor:
        - Used by built-in tasks like Cartpole, Ant, etc.
        - For now we send zeros unless a policy is connected.

        2. Scenic-controlled robot commands:
        - Used by Scenic behaviors like KeepMoving on Create3.
        - Buffered by LabBackend.applyRobotControl and applied directly to the
            corresponding Isaac Lab Articulation before env.step(...).
        """
        import traceback

        self._pending_robot_commands = {}

        try:
            # This calls action.applyTo(obj, self), which calls obj.move(self, ...).
            super().executeActions(allActions)
            self._pending_lab_action = self.scenicActionsToLabAction(allActions)
        except Exception as exc:
            print(
                f"[SCENIC ISAAC LAB ERROR] Exception while applying Scenic actions at step "
                f"{self._step_count}: {type(exc).__name__}: {exc}",
                flush=True,
            )
            traceback.print_exc()
            raise

    def scenicActionsToLabAction(self, allActions):
        """Map Scenic actions into an Isaac Lab action tensor.

        This returns a zero action tensor with the exact shape Isaac Lab expects,
        including the num_envs dimension.
        """
        if self.env is None:
            return None

        import torch

        env = self.env.unwrapped if hasattr(self.env, "unwrapped") else self.env

        if hasattr(env, "action_manager") and hasattr(env.action_manager, "action"):
            return torch.zeros_like(env.action_manager.action)

        action_space = getattr(self.env, "action_space", None)
        if action_space is None:
            return None

        num_envs = int(getattr(env, "num_envs", 1))
        shape = tuple(action_space.shape)

        if len(shape) == 1:
            shape = (num_envs, *shape)

        return torch.zeros(shape, device=env.device)

    def step(self):
        """Step the Isaac Lab environment once."""
        if self.env is None:
            return

        import traceback

        import torch

        try:
            self._resetEnvOnce()

            # Apply Scenic-controlled robot commands before stepping the Lab env.
            self._applyPendingRobotCommands()

            action = self._pending_lab_action
            if action is None:
                action = self.scenicActionsToLabAction([])

            with torch.inference_mode():
                self._last_step_output = self.env.step(action)

            self._pending_lab_action = None
            self._pending_robot_commands = {}
            self._step_count += 1

            if self.debug_lifecycle and self._step_count % 100 == 0:
                print(
                    f"[SCENIC LAB DEBUG] step={self._step_count}, "
                    f"sim_time={self._step_count * float(self.timestep):.3f}s"
                )

        except Exception as exc:
            print(
                f"[SCENIC LAB ERROR] Exception during Isaac Lab step "
                f"{self._step_count}: {type(exc).__name__}: {exc}",
                flush=True,
            )
            traceback.print_exc()
            raise

    def _applyPendingRobotCommands(self):
        """Apply buffered Scenic robot commands to Isaac Lab articulations."""
        if not self._pending_robot_commands:
            return

        for _, (obj, command) in self._pending_robot_commands.items():
            controller = getattr(obj, "wheelController", None)

            if controller == "differential":
                asset = self._assetForScenicObject(obj)
                self.backend.applyDifferentialDriveCommand(asset, obj, command)
            elif callable(getattr(obj, "control", None)):
                self.backend.applyArticulationAction(self, obj, obj.control(command))
            else:
                if self.debug_lifecycle:
                    print(
                        "[SCENIC ISAAC LAB DEBUG] Unsupported wheeled controller:",
                        controller,
                        "for",
                        getattr(obj, "name", obj),
                    )

    def _resetEnvOnce(self):
        if self.env is None or self._has_reset:
            return
        self.env.reset()
        self._has_reset = True

    def getProperties(self, obj, properties):
        """Read Scenic-requested properties from Isaac Lab asset buffers."""
        if not getattr(obj, "physics", False):
            return {prop: getattr(obj, prop) for prop in properties}

        if self.env is None:
            defaults = self._defaultPhysicsValues()
            return {prop: defaults[prop] for prop in properties}

        asset = self._assetForScenicObject(obj)
        if asset is None:
            defaults = self._defaultPhysicsValues()
            return {prop: defaults[prop] for prop in properties}

        values = self._physicsValuesFromAsset(asset, env_id=0)
        return {prop: values[prop] for prop in properties}

    def _assetForScenicObject(self, obj):
        scenic_name = getattr(obj, "name", None)
        if scenic_name is None:
            return None

        asset_name = self._object_name_to_asset_name.get(scenic_name)
        if asset_name is None:
            asset_name = self.backend._safeAssetName(scenic_name)

        env = self.env.unwrapped if hasattr(self.env, "unwrapped") else self.env
        return env.scene[asset_name]

    def _physicsValuesFromAsset(self, asset, env_id: int = 0):
        data = getattr(asset, "data", None)
        if data is None:
            return self._defaultPhysicsValues()

        pos = self.backend._tensorRow(
            getattr(data, "root_pos_w", None), env_id, default=(0.0, 0.0, 0.0)
        )
        quat = self.backend._tensorRow(
            getattr(data, "root_quat_w", None), env_id, default=(1.0, 0.0, 0.0, 0.0)
        )
        lin_vel = self.backend._tensorRow(
            getattr(data, "root_lin_vel_w", None), env_id, default=(0.0, 0.0, 0.0)
        )
        ang_vel = self.backend._tensorRow(
            getattr(data, "root_ang_vel_w", None), env_id, default=(0.0, 0.0, 0.0)
        )

        yaw, pitch, roll = self.backend.isaacQuatToScenicEulerAngles(quat)

        speed = math.sqrt(sum(v * v for v in lin_vel))
        angular_speed = math.sqrt(sum(v * v for v in ang_vel))

        return dict(
            position=Vector(*pos),
            velocity=Vector(*lin_vel),
            speed=speed,
            angularSpeed=angular_speed,
            angularVelocity=Vector(*ang_vel),
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )

    def _defaultPhysicsValues(self):
        return dict(
            position=Vector(0, 0, 0),
            velocity=Vector(0, 0, 0),
            speed=0.0,
            angularSpeed=0.0,
            angularVelocity=Vector(0, 0, 0),
            yaw=0.0,
            pitch=0.0,
            roll=0.0,
        )

    def destroy(self):
        if self.debug_lifecycle:
            result = getattr(self, "result", None)
            if result is None:
                print(
                    "[SCENIC ISAAC LAB DEBUG] destroy called before Simulation.result was set."
                )
            else:
                print(
                    "[SCENIC ISAAC LAB DEBUG] simulation ended:",
                    "terminationType=",
                    getattr(result, "terminationType", None),
                    "terminationReason=",
                    getattr(result, "terminationReason", None),
                )

        if self.env is not None and self._owns_env:
            self.env.close()
            self.env = None
