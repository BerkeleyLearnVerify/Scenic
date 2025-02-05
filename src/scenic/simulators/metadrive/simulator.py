"""Simulator interface for MetaDrive."""

try:
    from metadrive.component.traffic_participants.pedestrian import Pedestrian
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
except ImportError as e:
    raise ModuleNotFoundError(
        'Metadrive scenarios require the "metadrive" package'
    ) from e

import logging
import sys
import time

from scenic.core.simulators import SimulationCreationError
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
)
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.simulators.metadrive.actions import *
import scenic.simulators.metadrive.utils as utils


class MetaDriveSimulator(DrivingSimulator):
    """Implementation of `Simulator` for MetaDrive."""

    def __init__(
        self,
        timestep=0.1,
        render=True,
        render3D=False,
        sumo_map=None,
        real_time=True,
    ):
        super().__init__()
        self.render = render
        self.render3D = render3D if render else False
        self.scenario_number = 0
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.real_time = real_time
        (
            self.center_x,
            self.center_y,
            self.offset_x,
            self.offset_y,
            self.sumo_map_boundary,
        ) = self._get_map_parameters()

    def _get_map_parameters(self):
        """Helper method to extract map parameters."""
        # Use utility function to get the map parameters
        return utils.getMapParameters(self.sumo_map)

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
            real_time=self.real_time,
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
        real_time,
        center_x,
        center_y,
        offset_x,
        offset_y,
        sumo_map_boundary,
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
        self.real_time = real_time
        self.center_x = center_x
        self.center_y = center_y
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.sumo_map_boundary = sumo_map_boundary
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()

    def createObjectInSimulator(self, obj):
        """
        Create an object in the MetaDrive simulator.

        If it's the first object, it initializes the client and sets it up for the ego car.
        For additional cars and pedestrians, it spawns objects using the provided position and heading.
        """

        converted_position = utils.scenicToMetaDrivePosition(
            obj.position, self.center_x, self.center_y, self.offset_x, self.offset_y
        )
        converted_heading = utils.scenicToMetaDriveHeading(obj.heading)

        if not self.defined_ego:
            decision_repeat = math.ceil(self.timestep / 0.02)
            physics_world_step_size = self.timestep / decision_repeat

            # Initialize the simulator with ego vehicle
            self.client = utils.DriveEnv(
                dict(
                    decision_repeat=decision_repeat,
                    physics_world_step_size=physics_world_step_size,
                    use_render=self.render if self.render3D else False,
                    vehicle_config={
                        "spawn_position_heading": [
                            converted_position,
                            converted_heading,
                        ],
                    },
                    use_mesh_terrain=self.render3D,
                    log_level=logging.CRITICAL,
                )
            )
            self.client.config["sumo_map"] = self.sumo_map
            self.client.reset()

            # Assign the MetaDrive actor to the ego
            metadrive_objects = self.client.engine.get_objects()
            if metadrive_objects:
                ego_metaDriveActor = list(metadrive_objects.values())[0]
                obj.metaDriveActor = ego_metaDriveActor
                self.defined_ego = True
                return obj.metaDriveActor
            else:
                raise SimulationCreationError(
                    f"Unable to initialize MetaDrive ego vehicle for object {obj}"
                )

        # For additional cars
        if obj.isCar:
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                DefaultVehicle,
                vehicle_config=dict(),
                position=converted_position,
                heading=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor
            return metaDriveActor

        # For pedestrians
        if obj.isPedestrian:
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                Pedestrian,
                position=converted_position,
                heading_theta=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor
            return metaDriveActor

        # If the object type is unsupported, raise an error
        raise SimulationCreationError(
            f"Unsupported object type: {type(obj)} for object {obj}."
        )

    def executeActions(self, allActions):
        """Execute actions for all vehicles in the simulation."""
        super().executeActions(allActions)

        # Apply control updates to vehicles
        for idx, obj in enumerate(self.scene.objects):
            if obj.isCar and idx != 0:  # Skip ego vehicle (it is handled separately)
                action = obj.collectAction()
                obj.metaDriveActor.before_step(action)
                obj.resetControl()

    def step(self):
        start_time = time.monotonic()

        # Special handling for the ego vehicle
        ego_obj = self.scene.objects[0]
        if ego_obj.isCar:
            action = ego_obj.collectAction()
            o, r, tm, tc, info = self.client.step(action)  # Apply action in the simulator
            ego_obj.resetControl()

        # Render the scene in 2D if needed
        if self.render and not self.render3D:
            film_size = utils.calculateFilmSize(self.sumo_map_boundary, scaling=5)
            self.client.render(
                mode="topdown", semantic_map=True, film_size=film_size, scaling=5
            )

        # If real-time synchronization is enabled, sleep to maintain real-time pace
        if self.real_time:
            end_time = time.monotonic()
            elapsed_time = end_time - start_time
            if elapsed_time < self.timestep:
                time.sleep(self.timestep - elapsed_time)

    def destroy(self):
        if self.client and self.client.engine:
            object_ids = list(self.client.engine._spawned_objects.keys())
            self.client.engine.agent_manager.clear_objects(object_ids)
            self.client.close()

        super().destroy()

    def getProperties(self, obj, properties):
        metaDriveActor = obj.metaDriveActor
        position = utils.metadriveToScenicPosition(
            metaDriveActor.position,
            self.center_x,
            self.center_y,
            self.offset_x,
            self.offset_y,
        )
        velocity = Vector(*metaDriveActor.velocity, 0)
        speed = metaDriveActor.speed
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
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.13, K_D=0.3, K_I=0.05, dt=dt)
            # Good for Town 07
            # lat_controller = PIDLateralController(K_P=0.15, K_D=0.69, K_I=0.03, dt=dt)
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
