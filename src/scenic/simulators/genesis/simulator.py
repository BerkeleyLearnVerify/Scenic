"""Simulator interface for Genesis."""

import genesis as gs
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
    def __init__(self, *, genesis_options):
        super().__init__()
        self.genesis_options = genesis_options
        
    def createSimulation(self, scene, timestep, **kwargs):
        return GenesisSimulation(scene, timestep=timestep, genesis_options=self.genesis_options, **kwargs)

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

        timestep = 0.01 if timestep is None else timestep
        self.genesis_options = genesis_options

        # Optional batched initial scenes provided by simulator override
        if "batched_scenes" in kwargs:
            self.batched_scenes = kwargs['batched_initial_scenes']
        else:
            self.batched_scenes = [scene]
        
        # Initialize agents list before calling parent constructor
        self.agents = []

        # Create temporary directory for storing meshes
        self.tmpMeshDir = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)

        # Call parent constructor
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        init_options = {k: v for k, v in self.genesis_options.items() 
                       if k in ['backend', 'precision', 'debug', 'logging_level']}
        scene_options = {k: v for k, v in self.genesis_options.items() 
                        if k not in init_options}
        
        # Initialize Genesis if not already initialized
        if not gs._initialized:
            gs.init(**init_options)

        # Batch-related options
        batch_size = 1
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

        # Call parent setup to create objects (creates entities)
        super().setup()

        # Build the Genesis scene after all objects are added
        assert batch_size > 0
        self.gs_scene.build(n_envs=batch_size, env_spacing=env_spacing)

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
    
    # TODO: raise exception if called after scene has been built  
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
        self.gs_scene.step()

    def getProperties(self, obj, properties):
        entity = obj.genesis_entity

        # Get current position (base link position)
        if any(prop in properties for prop in ['position', 'elevation']):
            pos_tensor = entity.get_pos()
            # Convert to numpy and extract first environment if multi-env
            pos_array = pos_tensor.detach().cpu().numpy()
            if pos_array.ndim > 1:
                pos_array = pos_array[0]
            position = Vector(float(pos_array[0]), float(pos_array[1]), float(pos_array[2]))

        # Get current velocity
        if any(prop in properties for prop in ['velocity', 'speed']):
            vel_tensor = entity.get_vel()
            vel_array = vel_tensor.detach().cpu().numpy()
            if vel_array.ndim > 1:
                vel_array = vel_array[0]
            velocity = Vector(float(vel_array[0]), float(vel_array[1]), float(vel_array[2]))
            speed = float(velocity.norm())
        
        # Get current angular velocity
        if any(prop in properties for prop in ['angularVelocity', 'angularSpeed']):
            ang_tensor = entity.get_ang()
            ang_array = ang_tensor.detach().cpu().numpy()
            if ang_array.ndim > 1:
                ang_array = ang_array[0]
            angularVelocity = Vector(float(ang_array[0]), float(ang_array[1]), float(ang_array[2]))
            angularSpeed = float(angularVelocity.norm())
 
        # Get current orientation (quaternion -> Euler angles)
        if any(prop in properties for prop in ['yaw', 'pitch', 'roll']):
            quat_tensor = entity.get_quat()
            quat_array = quat_tensor.detach().cpu().numpy()
            if quat_array.ndim > 1:
                quat_array = quat_array[0]
            
            # Genesis quaternion format: [w, x, y, z]
            w, x, y, z = float(quat_array[0]), float(quat_array[1]), float(quat_array[2]), float(quat_array[3])
            
            # Convert to Euler angles (in radians)
            import math
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
            roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        values = {
            "position": position,
            "velocity": velocity,
            "speed": speed,
            "angularVelocity": angularVelocity,
            "angularSpeed": angularSpeed,
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
        }

        return values
