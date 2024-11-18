try:
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
except ImportError as e:
    raise ModuleNotFoundError(
        'Metadrive scenarios require the "metadrive" package'
    ) from e

import logging
import sys

import numpy as np

from scenic.core.simulators import SimulationCreationError
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.simulators.metadrive.actions import *
import scenic.simulators.metadrive.utils as utils

from .utils import DriveEnv


class MetaDriveSimulator(DrivingSimulator):
    def __init__(
        self,
        timestep=0.1,
        render=True,
        render3D=False,
        sumo_map=None,
        center_x=None,
        center_y=None,
    ):
        super().__init__()
        self.render = render
        self.render3D = render3D if render else False  # Enforce no rendering if render=False
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
            render3D=self.render3D,
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
        render3D,
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
        self.render3D = render3D
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
            # Print step-by-step discrepancies
            # print(f"Step Scenic position: {obj.position}")
            # print(f"Step MetaDrive position: {obj.metaDriveActor.last_position}")
            # print(f"Step Scenic heading: {obj.heading}")
            # print(f"Step MetaDrive heading: {obj.metaDriveActor.last_heading_dir}")
            # Add the road direction check here after initialization
            # if self.scenario_number == 1:  # Check on the first scenario step, for example
            #     # check road direction!!!!
            #     pass
        if self.render and not self.render3D:
            # self.client.render(mode="topdown", scaling=1.5, camera_position=(0,0))
            self.client.render(mode="topdown")

    def executeActions(self, allActions):
        super().executeActions(allActions)

    def createObjectInSimulator(self, obj):
        if not self.defined_ego:
            self.client = DriveEnv(
                dict(
                    use_render=self.render if self.render3D else False,
                    vehicle_config={
                        "spawn_position_heading": [
                            utils.scenicToMetaDrivePosition(
                                obj.position, self.center_x, self.center_y
                            ),
                            obj.heading,
                        ]
                    },
                    use_mesh_terrain=self.render3D,
                    log_level=logging.CRITICAL,
                )
            )
            self.client.config["sumo_map"] = self.sumo_map
            self.client.reset()

            metadrive_objects = self.client.engine.get_objects()
            for _, v in metadrive_objects.items():
                metaDriveActor = v
                obj.metaDriveActor = metaDriveActor

                # Initial positions and headings
                # print(f"Initial Scenic position: {obj.position}")
                # print(f"Initial MetaDrive position: {metaDriveActor.last_position}")
                # Print statements for debugging position
                # print(f"Scenic Position (absolute): {obj.position}")
                # meta_position = utils.scenicToMetaDrivePosition(obj.position, self.center_x, self.center_y)
                # print(f"Converted MetaDrive Position (relative): {meta_position}")
                print(f"Initial Scenic heading: {obj.heading}")
                print(f"Initial MetaDrive heading: {metaDriveActor.heading_theta}")

                self.defined_ego = True
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
        position = utils.metadriveToScenicPosition(
            metaDriveActor.last_position, self.center_x, self.center_y
        )
        velocity = utils.metadriveToScenicPosition(
            metaDriveActor.last_velocity, self.center_x, self.center_y
        )
        speed = metaDriveActor.last_speed
        angularSpeed = 0
        angularVelocity = utils.metadriveToScenicPosition(
            (0, 0), self.center_x, self.center_y
        )
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
