"""Simulator interface for Genesis."""

import genesis as gs
import collections
import torch
import ast
from scenic.core.simulators import Simulator, Simulation
from scenic.core.scenarios import Scenario
from scenic.core.vectors import Vector
from scenic.core.type_support import toOrientation
import numpy as np
import scenic.core.dynamics as dynamics
import tempfile

class GenesisSimulator(Simulator):
    def __init__(self, *, timestep=0.01, genesis_options):
        super().__init__()
        self.timestep = timestep
        self.genesis_options = genesis_options
        
    def createSimulation(self, scenes, timestep, **kwargs):
        return GenesisSimulation(scenes, timestep=timestep if timestep else self.timestep, genesis_options=self.genesis_options, **kwargs)

    def simulateFromScenario(
        self,
        scenario: Scenario,
        *,
        maxSteps=None,
        maxIterations=1,
        timestep=None,
        verbosity=None,
        raiseGuardViolations=False,
        replay=None,
        enableReplay=True,
        enableDivergenceCheck=False,
        divergenceTolerance=0,
        continueAfterDivergence=False,
        allowPickle=False,
    ):
        """Batched scene sampling support for Genesis.

        If `batch_size>0` is provided in simulator options, sample that many
        scenes via Scenario.generateBatch and initialize a single Genesis
        scene with n_envs=batch_size. Otherwise, fall back to single-scene.
        """
        if verbosity is None:
            verbosity = 0
        batch_size = self.genesis_options.get('batch_size', 1)
        if batch_size < 1:
            raise ValueError("Batch size must be greater than or equal to 1.")

        # Sample B scenes using Scenic (preserving Scenic randomness per env), 
        # also calls the rest of the Scenario generation code
        scenes, _ = scenario.generateBatch(batch_size, verbosity=verbosity)

        sim = self.createSimulation(
            scenes[0],
            maxSteps=maxSteps,
            name=1,
            timestep=timestep,
            verbosity=verbosity,
            batched_initial_scenes=scenes,
        )
        return sim

class GenesisSimulation(Simulation):
    def __init__(
        self,
        scene,
        *,
        timestep,
        genesis_options,
        **kwargs):

        assert isinstance(scene, collections.abc.Sequence)
        self.batched_scenes = tuple(scene)

        self.genesis_options = genesis_options
        self.actions = None

        # Create temporary directory for storing meshes
        self.tmpMeshDir = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)

        # Call parent constructor
        super().__init__(scene[0], timestep=timestep, **kwargs)

    def setup(self):
        init_options = {k: v for k, v in self.genesis_options.items() 
                       if k in ['backend', 'precision', 'debug', 'logging_level']}
        scene_options = {k: v for k, v in self.genesis_options.items() 
                        if k not in init_options}
        
        # Initialize Genesis if not already initialized
        if not gs._initialized:
            gs.init(**init_options)

        # Batch-related options
        env_spacing = scene_options.get('env_spacing', (5.0, 5.0))
        # Create Genesis scene with proper physics settings
        self.gs_scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
            res=(1280, 720),
            camera_pos=(5, 0.0, 5),
            camera_lookat=(0.0, 0.0, 0.5),
            camera_fov=40,
            ),
            profiling_options=gs.options.ProfilingOptions(
                show_FPS=False
            ),
            sim_options=gs.options.SimOptions(
                dt=self.timestep,
                gravity=(0.0, 0.0, -9.81),
                substeps=scene_options["substeps"]
            ),
            show_viewer=scene_options['show_viewer']
        )

        # Call parent setup
        super().setup()

        # Build the Genesis scene after all objects are added
        self.gs_scene.build(n_envs=len(self.batched_scenes), env_spacing=env_spacing)

        for obj in self.objects:
            if hasattr(obj, "genesisStartDynamicSimulation"):
                obj.genesisStartDynamicSimulation() 

        # Apply Scenic initial states to each of the Genesis envs
        for obj_index, obj in enumerate(self.scene.objects):
            entity = obj.genesis_entity
            for env_idx, sc in enumerate(self.batched_scenes):
                sobj = sc.objects[obj_index]
                pos = tuple(sobj.position + sobj.positionOffset)
                entity.set_pos(pos, envs_idx=[env_idx], zero_velocity=True)
                targetOrientation = (sobj.orientation * toOrientation(sobj.orientationOffset))
                targetOrientation.r.as_quat(scalar_first=True)
                entity.set_quat(targetOrientation.r.as_quat(scalar_first=True), envs_idx=[env_idx], zero_velocity=True)

    def destroy(self):
        self.tmpMeshDir.cleanup()
        return super().destroy()

    def createObjectInSimulator(self, obj):
        # Create Genesis morph using the object's makeMorph method
        morph = obj.makeMorph()

        # Create Genesis material
        # TODO: Customizable density, friction, and material in model
        density = getattr(obj, 'density', 200.0)
        friction = getattr(obj, 'friction', 1.0)
        material = gs.materials.Rigid(rho=density, friction=friction)

        # Create surface with color if specified
        surface = None
        if hasattr(obj, 'color') and obj.color is not None:
            surface = gs.surfaces.Rough(
                diffuse_texture=gs.textures.ColorTexture(
                    color=tuple(obj.color)
                ),
            )

        # Create and add Genesis entity to scene
        entity = self.gs_scene.add_entity(
            morph=morph,
            material=material,
            surface=surface
        )

        # Store reference to Genesis entity
        obj.genesis_entity = entity

    def step(self):
        self.executeGymAction()
        self.gs_scene.step()

    def executeGymAction(self):
        # TODO: Add support for actions on all objects
        assert self.actions.shape[0] == 1 and self.actions.shape[1] == len(self.batched_scenes)
        self.objects[0].executeActions(self.actions[0])

    def getProperties(self, obj, properties):
        # TODO: Remove loop in updateObjects
        values = {
            "position": obj.position,
            "velocity": obj.velocity,
            "speed": obj.speed,
            "angularVelocity": obj.angularVelocity,
            "angularSpeed": obj.angularSpeed,
            "yaw": obj.yaw,
            "pitch": obj.pitch,
            "roll": obj.roll,
        }

        return values
