try:
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
except ImportError as e:
    raise ModuleNotFoundError(
        'Metadrive scenarios require the "metadrive" package'
    ) from e

import logging
import sys

from scenic.core.simulators import SimulationCreationError
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.simulators.metadrive.actions import *
import scenic.simulators.metadrive.utils as utils
import numpy as np

from .utils import DriveEnv


class MetaDriveSimulator(DrivingSimulator):
    def __init__(
        self,
        timestep=0.1,
        render=True,
        sumo_map=None,
        center_x=None,
        center_y=None,
    ):
        super().__init__()
        self.render = render
        self.scenario_number = 0
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.center_x = center_x
        self.center_y = center_y

    def createSimulation(self, scene, *, timestep, **kwargs):
        if timestep is not None and timestep != self.timestep:
            raise RuntimeError(
                "cannot customize timestep for individual MetaDrive simulations; "
                "set timestep when creating the MetaDriveSimulator instead"
            )
        self.scenario_number += 1
        return MetaDriveSimulation(
            scene,
            render=self.render,
            scenario_number=self.scenario_number,
            timestep=self.timestep,
            sumo_map=self.sumo_map,
            center_x=self.center_x,
            center_y=self.center_y,
            **kwargs,
        )

    def destroy(self):
        super().destroy()


class MetaDriveSimulation(DrivingSimulation):
    def __init__(
        self,
        scene,
        render,
        scenario_number,
        timestep,
        sumo_map,
        center_x,
        center_y,
        **kwargs,
    ):
        # NOTE: MetaDrive requires at least one agent to be defined per simulation run
        try:
            if len(scene.objects) == 0:
                raise SimulationCreationError(
                    "The Metadrive interface requires you to define at least one Scenic object."
                )
        except SimulationCreationError as e:
            print(e)
            sys.exit(1)

        self.render = render
        self.scenario_number = scenario_number
        self.defined_ego = False
        self.client = None
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.center_x = center_x
        self.center_y = center_y
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()

    def step(self):
        if len(self.scene.objects) > 0:
            obj = self.scene.objects[0]
            action = obj.metaDriveActor.last_current_action[-1]
            o, r, tm, tc, info = self.client.step(action)
    
    def executeActions(self, allActions):
        super().executeActions(allActions)

    def createObjectInSimulator(self, obj):
        if not self.defined_ego:
            breakpoint()
            '''
            Object Heading appears to be the same each time:
            (Pdb) obj.heading
            -1.5706744740097156

            THERE IS SOMETHING WRONG WITH HOW METADRIVE WILL DEFINE WHAT DIRECTION THE CAR
            SHOULD BE FACING
            (Pdb) self.client.engine.get_objects()['d9abbad9-7c93-450f-b662-78569a9cfad1'].last_heading_dir
            (1.0, 0.0)
            (Pdb) obj.heading
            -1.5712431204764379

            Relying on the Scenic probabilistic scene to generate the heading direction
            (Pdb) obj.heading
            -3.1410703658320345
            (Pdb) self.client.engine.get_objects()
            {'a2fc38fa-a858-41f9-a43a-ea16e83a9d11': DefaultVehicle, ID:a2fc38fa-a858-41f9-a43a-ea16e83a9d11}
            (Pdb) self.client.engine.get_objects()['a2fc38fa-a858-41f9-a43a-ea16e83a9d11'].last_heading_dir
            (-0.9999998634905004, -0.0005225121820975668)

            (Pdb) self.client.engine.get_objects()['d55626f4-7200-4ffc-af87-f5594195d5eb'].last_heading_dir
            (8.12594588518524e-05, 0.9999999966984502)
            (Pdb) self.obj.heading
            *** AttributeError: 'MetaDriveSimulation' object has no attribute 'obj'
            (Pdb) self.object.heading
            *** AttributeError: 'MetaDriveSimulation' object has no attribute 'object'
            (Pdb) self.objects[0].heading
            1.570715067335482
            '''
            self.client = DriveEnv(
                dict(
                    use_render=self.render,
                    vehicle_config={
                        "spawn_position_heading": [
                            utils.scenicToMetaDrivePosition(
                                obj.position, self.center_x, self.center_y
                            ),
                            # DEFINED FROM MANUAL CALCULATIONS
                            # (195.3623156838, -143.1532659478),
                            # obj.heading,
                            # np.pi / 2,
                            obj.heading,
                        ]
                    },
                    use_mesh_terrain=True,
                    log_level=logging.CRITICAL,
                )
            )
            self.client.config["sumo_map"] = self.sumo_map
            self.client.reset()

            metadrive_objects = self.client.engine.get_objects()
            for _, v in metadrive_objects.items():
                metaDriveActor = v
                obj.metaDriveActor = metaDriveActor
                return metaDriveActor

        if type(obj).__name__ == "Car":
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                DefaultVehicle,
                vehicle_config=dict(),
                position=utils.scenicToMetaDrivePosition(
                    obj.position, self.center_x, self.center_y
                ),
                heading=obj.heading,
            )
            obj.metaDriveActor = metaDriveActor

        return metaDriveActor

    def destroy(self):
        if self.client:
            object_ids = list(self.client.engine._spawned_objects.keys())
            self.client.engine.agent_manager.clear_objects(object_ids)
            self.client.close()

        super().destroy()

    def getProperties(self, obj, properties):
        metaDriveActor = obj.metaDriveActor
        position = utils.metadriveToScenicPosition(metaDriveActor.last_position, self.center_x, self.center_y)
        velocity = utils.metadriveToScenicPosition(metaDriveActor.last_velocity, self.center_x, self.center_y)
        speed = metaDriveActor.last_speed
        angularSpeed = 0
        angularVelocity = utils.metadriveToScenicPosition((0, 0), self.center_x, self.center_y)
        yaw, pitch, _ = obj.parentOrientation.globalToLocalAngles(
            metaDriveActor.last_heading_dir[0], metaDriveActor.last_heading_dir[1], 0
        )

        yaw = yaw
        pitch = pitch
        roll = 0
        elevation = 0

        values = dict(
            position=position,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            elevation=elevation,
        )

        return values
