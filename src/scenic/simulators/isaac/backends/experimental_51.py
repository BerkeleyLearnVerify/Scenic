from dataclasses import dataclass, field

from scenic.simulators.isaac.backends.base import IsaacBackend
from scenic.simulators.isaac.backends.experimental_60 import Experimental60Backend


@dataclass
class Experimental51World:
    core_world: object
    app: object
    timestep: float
    objects: dict = field(default_factory=dict)

    def get_object(self, name):
        return self.objects[name]


class Experimental51Backend(Experimental60Backend):
    """Isaac Sim 5.1 backend using Core Experimental wrappers."""

    name = "experimental_51"

    def create_world(self, timestep):
        from isaacsim.core.api import World

        core_world = World(
            stage_units_in_meters=1.0,
            physics_dt=timestep,
            rendering_dt=timestep,
        )
        return Experimental51World(
            core_world=core_world,
            app=self._simulation_app,
            timestep=timestep,
        )

    def run_coroutine(self, coro):
        return IsaacBackend.run_coroutine(self, coro)

    def open_environment_stage(self, usd_path):
        import isaacsim.core.experimental.utils.stage as stage_utils

        opened, stage = self._open_stage(stage_utils, usd_path)
        if not opened:
            return False
        stage.SetEditTarget(stage.GetSessionLayer())
        return True

    def _open_stage_for_conversion(self, usd_path):
        import isaacsim.core.experimental.utils.stage as stage_utils

        opened, _ = self._open_stage(stage_utils, usd_path)
        return opened

    def _open_stage(self, stage_utils, usd_path):
        result = stage_utils.open_stage(usd_path)
        if isinstance(result, tuple):
            opened, stage = result
        else:
            opened = bool(result)
            stage = stage_utils.get_current_stage() if opened else None
        return opened, stage

    def initialize_physics(self, world, objects):
        world.core_world.initialize_physics()

    def play_world(self, world):
        world.core_world.play()

    def step_world(self, world):
        world.core_world.step()

    def stop_and_clear_world(self, world):
        world.core_world.stop()
        world.core_world.clear()
        world.objects.clear()

    def release_world(self, world):
        from isaacsim.core.api import World

        World.clear_instance()

    def add_object(self, world, obj, *, scenic_obj=None):
        world.objects[scenic_obj.name] = obj
