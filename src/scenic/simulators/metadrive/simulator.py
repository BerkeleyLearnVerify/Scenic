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
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
)
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
        offset_x=0,
        offset_y=0,
        sumo_map_boundary=None,
    ):
        super().__init__()
        self.render = render
        self.render3D = (
            render3D if render else False
        )  # Enforce no rendering if render=False
        self.scenario_number = 0
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.center_x = center_x
        self.center_y = center_y
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.sumo_map_boundary = sumo_map_boundary

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
            offset_x=self.offset_x,
            offset_y=self.offset_y,
            sumo_map_boundary=self.sumo_map_boundary,
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
        offset_x=0,
        offset_y=0,
        sumo_map_boundary=None,
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
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.sumo_map_boundary = sumo_map_boundary
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()

    def step(self):
        print("IN STEP")

        # decision_repeat = math.ceil(self.timestep / 0.02)
        # physics_world_step_size = self.timestep / decision_repeat

        # self.client.config["decision_repeat"] = decision_repeat
        # self.client.config["physics_world_step_size"] = physics_world_step_size

        if len(self.scene.objects) > 0:
            obj = self.scene.objects[0]
            print(f"Before Action - Velocity of EGO: {obj.metaDriveActor.velocity}")
            action = obj.metaDriveActor.last_current_action[-1]
            print("ACTION FOR EGO IN METADRIVE IS: ", action)
            # if action == [0, -1]:
            #     breakpoint()
            o, r, tm, tc, info = self.client.step(action)
            print(f"After Action - Velocity of EGO: {obj.metaDriveActor.velocity}")
            # print(f"Speed of EGO: {obj.metaDriveActor.speed_km_h}")
            # print(f"Position of EGO: {obj.metaDriveActor.position}")

        if self.render and not self.render3D:
            scaling = 5
            film_size = utils.calculateFilmSize(self.sumo_map_boundary, scaling)
            self.client.render(
                mode="topdown", semantic_map=True, film_size=film_size, scaling=scaling
            )
        # breakpoint()

    def executeActions(self, allActions):
        """Execute actions for all vehicles in the simulation."""
        super().executeActions(allActions)

        # Iterate through all agents in the scene
        for obj in self.scene.objects:
            if hasattr(obj, "metaDriveActor") and obj.metaDriveActor is not None:
                # Apply the accumulated control inputs using before_step
                obj.applyControl()

                # print(
                #     f"MetaDriveActor State - Steering: {obj.metaDriveActor.last_current_action[-1][0]}, "
                #     f"Throttle/Brake: {obj.metaDriveActor.last_current_action[-1][1]}"
                # )

                # breakpoint()

                # Reset control inputs after they are applied
                obj.resetControl()

    def createObjectInSimulator(self, obj):
        """
        Create an object in the MetaDrive simulator.

        If it's the first object, it initializes the client and sets it up for the ego car.
        For additional cars, it spawns objects using the provided position and heading.
        """
        # Convert position and heading from Scenic to MetaDrive
        converted_position = utils.scenicToMetaDrivePosition(
            obj.position, self.center_x, self.center_y, self.offset_x, self.offset_y
        )
        converted_heading = utils.scenicToMetaDriveHeading(obj.heading)

        if not self.defined_ego:
            # print("Initial Scenic Position: ", obj.position)
            # print(f"Converted Position: {converted_position}")

            # print("Scenic Heading: ", obj.heading)
            # print(f"Converted Heading: {converted_heading}")

            decision_repeat = math.ceil(self.timestep / 0.02)
            physics_world_step_size = self.timestep / decision_repeat

            # Initialize the simulator with ego vehicle
            self.client = DriveEnv(
                dict(
                    decision_repeat=decision_repeat,
                    physics_world_step_size=physics_world_step_size,
                    use_render=self.render if self.render3D else False,
                    vehicle_config={
                        "spawn_position_heading": [
                            converted_position,
                            converted_heading,
                        ],
                        # "enable_reverse": True,
                    },
                    use_mesh_terrain=self.render3D,
                    log_level=logging.CRITICAL,
                )
            )
            # self.client.config["max_brake_force"] = 100
            self.client.config["sumo_map"] = self.sumo_map
            self.client.reset()

            # Assign the MetaDrive actor to the ego
            metadrive_objects = self.client.engine.get_objects()
            if metadrive_objects:
                ego_metaDriveActor = list(metadrive_objects.values())[
                    0
                ]  # Assuming the ego is the first object
                obj.metaDriveActor = ego_metaDriveActor
                print("EGO CAR: ", obj.metaDriveActor)
                # breakpoint()
                # print("INITIAL METADRIVE POSITION: ", obj.metaDriveActor.position)
                self.defined_ego = True
                return obj.metaDriveActor

        # For additional cars
        if type(obj).__name__ == "Car":
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                DefaultVehicle,
                vehicle_config=dict(),
                position=converted_position,
                heading=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor
            print("OTHER CAR: ", obj.metaDriveActor)
            return metaDriveActor

        return None

    def destroy(self):
        print("------END POSITIONS----------")
        ego = self.scene.objects[0]
        scenic_position = ego.position
        print("SCENIC END POSITION: ", scenic_position)
        converted_position = utils.scenicToMetaDrivePosition(
            scenic_position, self.center_x, self.center_y, self.offset_x, self.offset_y
        )
        print("METADRIVE END POSITION: ", ego.metaDriveActor.position)
        print("CONVERTED POSITION: ", converted_position)

        if self.client:
            object_ids = list(self.client.engine._spawned_objects.keys())
            self.client.engine.agent_manager.clear_objects(object_ids)
            self.client.close()

        super().destroy()

    def getProperties(self, obj, properties):
        metaDriveActor = obj.metaDriveActor
        position = utils.metadriveToScenicPosition(
            metaDriveActor.last_position,
            self.center_x,
            self.center_y,
            self.offset_x,
            self.offset_y,
        )
        velocity = Vector(*metaDriveActor.last_velocity, 0)  # [x,y,z]

        # print("IN GET PROPERTIES")
        # print("METADRIVE ACTOR LAST VELOCITY: ", metaDriveActor.last_velocity)
        # print("METADRIVE TO SCENIC VEL: ", velocity)
        speed = metaDriveActor.last_speed
        angularSpeed = 0
        angularVelocity = utils.metadriveToScenicPosition(
            (0, 0), self.center_x, self.center_y, self.offset_x, self.offset_y
        )

        converted_heading = utils.metaDriveToScenicHeading(metaDriveActor.heading_theta)

        yaw, pitch, roll = obj.parentOrientation.globalToLocalAngles(
            converted_heading, 0, 0
        )
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

    def getLaneFollowingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            # lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            # lat_controller = PIDLateralController(K_P=0.05, K_D=0.05, K_I=0.02, dt=dt)
            lon_controller = PIDLongitudinalController(K_P=0.3, K_D=0.05, K_I=0.1, dt=dt)
            lat_controller = PIDLateralController(K_P=0.001, K_D=0.01, K_I=0.001, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getTurningControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.2, K_I=0.2, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.4, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getLaneChangingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.2, K_I=0.02, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.1, K_D=0.3, K_I=0.0, dt=dt)
        return lon_controller, lat_controller
