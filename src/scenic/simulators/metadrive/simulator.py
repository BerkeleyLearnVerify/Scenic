"""Simulator interface for MetaDrive."""

try:
    from metadrive.component.traffic_participants.pedestrian import Pedestrian
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
    from metadrive.policy.expert_policy import ExpertPolicy
    from metadrive.envs import MetaDriveEnv
    from metadrive.component.navigation_module.node_network_navigation import NodeNetworkNavigation
    from metadrive.component.navigation_module.edge_network_navigation import EdgeNetworkNavigation
    from metadrive.component.navigation_module.base_navigation import BaseNavigation 
    from metadrive.component.navigation_module.trajectory_navigation import TrajectoryNavigation

except ImportError as e:
    raise ModuleNotFoundError(
        "Metadrive is required. Please install the 'metadrive-simulator' package (and sumolib) or use scenic[metadrive]."
    ) from e

import logging
import sys
import time

from scenic.core.simulators import InvalidScenarioError, SimulationCreationError
from scenic.domains.driving.actions import *
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
)
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
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
        # print(f"SIMULAOTR SUMO MAP {self.sumo_map}")
        self.real_time = real_time
        self.scenic_offset, self.sumo_map_boundary = utils.getMapParameters(self.sumo_map)

        if self.render and not self.render3D:
            self.film_size = utils.calculateFilmSize(self.sumo_map_boundary, scaling=5)
        else:
            self.film_size = None

    def createSimulation(self, scene, *, timestep, **kwargs):
        self.scenario_number += 1
        return MetaDriveSimulation(
            scene,
            render=self.render,
            render3D=self.render3D,
            scenario_number=self.scenario_number,
            timestep=self.timestep,
            sumo_map=self.sumo_map,
            real_time=self.real_time,
            scenic_offset=self.scenic_offset,
            sumo_map_boundary=self.sumo_map_boundary,
            film_size=self.film_size,
            **kwargs,
        )


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
        scenic_offset,
        sumo_map_boundary,
        film_size,
        **kwargs,
    ):
        if len(scene.objects) == 0:
            raise InvalidScenarioError(
                "Metadrive requires you to define at least one Scenic object."
            )
        if not scene.objects[0].isCar:
            raise InvalidScenarioError(
                "The first object must be a car to serve as the ego vehicle in Metadrive."
            )

        self.render = render
        self.render3D = render3D
        self.scenario_number = scenario_number
        self.defined_ego = False
        self.client = None
        self.timestep = timestep
        self.sumo_map = sumo_map
        self.real_time = real_time
        self.scenic_offset = scenic_offset
        self.sumo_map_boundary = sumo_map_boundary
        self.film_size = film_size
        self.actions = dict()
        self.camera_position = (0, 0, 0)
        self.agent_configs = dict()
        self.learning_agents = [] # The names of the learning agents

        self.observation = None
        self.reward = None
        self.tm = None
        self.tc = None
        self.info = None
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        try:
            # TODO something other than try and except
            self.camera_position = utils.scenicToMetaDrivePosition(
                self.scene.params["camera_position"], self.scenic_offset
            )
        except KeyError:
            print("No camera_position param specified in scenic program, defaulting to (0, 0, 0)")

        
        for obj in self.scene.objects:
            if obj.is_agent:
                self.learning_agents.append(obj)

                converted_position = utils.scenicToMetaDrivePosition(
                    obj.position, self.scenic_offset
                )
                converted_heading = utils.scenicToMetaDriveHeading(obj.heading)

                self.agent_configs[obj.name] = dict(use_special_color=obj.use_special_color, 
                                               spawn_lane_index=None,
                                               spawn_position_heading=[converted_position, converted_heading],
                                               lane_line_detector=dict(num_lasers=4, 
                                                                       distance=20, 
                                                                       gaussian_noise=0.0, 
                                                                       dropout_prob=0.0)
                                                )
                                    

        super().setup()

    def createObjectInSimulator(self, obj):
        """
        Create an object in the MetaDrive simulator.

        If it's the first object, it initializes the client and sets it up for the ego car.
        For additional cars and pedestrians, it spawns objects using the provided position and heading.
        """
        converted_position = utils.scenicToMetaDrivePosition(
            obj.position, self.scenic_offset
        )
        converted_heading = utils.scenicToMetaDriveHeading(obj.heading)


        if not self.defined_ego:
            decision_repeat = math.ceil(self.timestep / 0.02)
            physics_world_step_size = self.timestep / decision_repeat
            # print(f"CONVERTED POS: {converted_position}")
            # Initialize the simulator with ego vehicle
            self.client = utils.DriveEnv(
                dict(
                    agent_configs=self.agent_configs,
                    decision_repeat=decision_repeat,
                    physics_world_step_size=physics_world_step_size,
                    use_render=self.render3D,
                    is_multi_agent=True,
                    # num_agents=2,
                    num_agents=len(self.learning_agents),
                    # vehicle_config={
                        # "spawn_position_heading": [
                            # converted_position,
                            # converted_heading,
                        # ],
                        # "lane_line_detector" : dict(num_lasers=4, distance=20, gaussian_noise=0.0, dropout_prob=0.0),
                    # },
                    use_mesh_terrain=self.render3D,
                    log_level=logging.CRITICAL,
                    # traffic_density=0.2
                    # agent_conigs=dict()
                )
            )

            self.client.config["sumo_map"] = self.sumo_map
            self.observation, self.info = self.client.reset()

            # Assign the MetaDrive actor to the ego
            # Recall that MetaDrive actor is of type metadrive.DefaultVehicle
            metadrive_objects = self.client.engine.get_objects()
            obj.metaDriveActor = list(metadrive_objects.values())[0]
            self.defined_ego = True
            return

        if obj.is_agent:
            obj.metaDriveActor = self.client.agents[obj.name]
            return

        # For additional cars
        elif obj.isVehicle:

            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                DefaultVehicle,
                vehicle_config=dict(lane_line_detector=dict(num_lasers=4, distance=20, gaussian_noise=0.0, dropout_prob=0.0)),
                position=converted_position,
                heading=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor # TODO this has set_pos and set_heading_theta functions
            # print(f"ENV AGENTS {self.client.agents['agent0']}")
            return

        # For pedestrians
        # elif obj.isPedestrian:
        elif obj.isPedestrian:
            metaDriveActor = self.client.engine.agent_manager.spawn_object(
                Pedestrian,
                position=converted_position,
                heading_theta=converted_heading,
            )
            obj.metaDriveActor = metaDriveActor
            return
 
        # If the object type is unsupported, raise an error
        raise SimulationCreationError(
            f"Unsupported object type: {type(obj)} for object {obj}."
        )

    def executeActions(self, allActions):
        """Execute actions for all vehicles in the simulation."""
        super().executeActions(allActions)

        # Apply control updates to vehicles and pedestrians
        for obj in self.scene.objects:  # Skip ego vehicle (it is handled separately)
            if obj.is_agent:
                continue
            # print("not agent")
            if obj.isVehicle:
                action = obj._collect_action()
                obj.metaDriveActor.before_step(action)
                obj._reset_control()
            else:
                # For Pedestrians
                if obj._walking_direction is None:
                    obj._walking_direction = utils.scenicToMetaDriveHeading(obj.heading)
                if obj._walking_speed is None:
                    obj._walking_speed = obj.speed
                direction = [
                    math.cos(obj._walking_direction),
                    math.sin(obj._walking_direction),
                ]
                obj.metaDriveActor.set_velocity(direction, obj._walking_speed)

    def step(self):
        start_time = time.monotonic()

        # Special handling for the ego vehicle
        # print(f"actions: {self.actions}")
        # ego_obj = self.scene.objects[0]
        
        # TODO this for loop is just for debug
        for obj in self.scene.objects:
            # print(f"has behavior???: {not obj.behavior}")
            if obj.is_agent and obj.behavior:
                self.actions[obj.name] = obj._collect_action() # TODO will have to go in the future...

        print(f"ACTION = {self.actions}")
        # print(f"Config: {self.client.config}")
        self.observation, self.reward, self.tm, self.tc, self.info = self.client.step(self.actions)  # Apply action in the simulator
        self.actions = dict()

        # Render the scene in 2D if needed
        if self.render and not self.render3D:
            self.client.render(
                mode="topdown", 
                semantic_map=True,
                film_size=self.film_size, 
                scaling=5,
                camera_position = self.camera_position[:2]
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
            if object_ids:
                self.client.engine.agent_manager.clear_objects(object_ids)
            self.client.close()

        super().destroy()

    def getProperties(self, obj, properties):
        # print(obj.reward)
        metaDriveActor = obj.metaDriveActor
        position = utils.metadriveToScenicPosition(
            metaDriveActor.position, self.scenic_offset
        )
        # print(f"Car Pos {position}")
        velocity = Vector(*metaDriveActor.velocity, 0)
        speed = metaDriveActor.speed
        md_ang_vel = metaDriveActor.body.getAngularVelocity()
        angularVelocity = Vector(*md_ang_vel)
        angularSpeed = math.hypot(*md_ang_vel)
        converted_heading = utils.metaDriveToScenicHeading(metaDriveActor.heading_theta)
        # yaw, pitch, roll = obj.parentOrientation.globalToLocalAngles(
            # converted_heading, 0, 0
        # )
        elevation = 0
        # if obj.name=='agent1' :
        # print(f"Name: {obj.name}, Yaw pitch roll: {yaw, pitch, roll}")
        # print(f"CONVERTED HEADING {converted_heading}")
        # print(f"Yaw degree {180 * yaw/(3.1415926535)}")

        values = dict(
            position=position,
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=angularVelocity,
            yaw=converted_heading,
            pitch=0,
            roll=0,
            elevation=elevation,
        )

        # values = dict(
            # position=Vector(0, 0, 0),
            # velocity=Vector(0, 0, 0),
            # speed=0,
            # angularSpeed=0,
            # angularVelocity=Vector(0, 0, 0),
            # yaw=0,
            # pitch=0,
            # roll=0,
            # elevation=0,
        # )

        return values

    def getLaneFollowingControllers(self, agent):
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.13, K_D=0.3, K_I=0.05, dt=dt)
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

    def get_obs(self):
        return self.observation

    def get_info(self):
        return self.info

    def get_reward(self):

        reward = dict()
        for agent in self.learning_agents:
            reward[agent.name] = agent.reward

        return reward
            
