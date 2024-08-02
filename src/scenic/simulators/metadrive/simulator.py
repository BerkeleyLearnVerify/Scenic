try:
    from metadrive.component.vehicle.vehicle_type import DefaultVehicle
    from metadrive.component.map.base_map import BaseMap
    from metadrive.component.map.pg_map import MapGenerateMethod
    from metadrive.examples.ppo_expert.numpy_expert import expert
    from metadrive.utils.opendrive.map_load import load_opendrive_map
    from metadrive.component.road_network.edge_road_network import OpenDriveRoadNetwork
    from metadrive.component.opendrive_block.opendrive_block import OpenDriveBlock
    from metadrive.policy.idm_policy import IDMPolicy
    from metadrive.utils.draw_top_down_map import draw_top_down_map
except ImportError as e:
    raise ModuleNotFoundError('Metadrive scenarios require the "metadrive" Python package') from e

from .utils import DriveEnv
from scenic.core.simulators import SimulationCreationError
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.simulators.metadrive.actions import *
import logging

import scenic.simulators.metadrive.utils as utils

class MetaDriveSimulator(DrivingSimulator):
    def __init__(
            self,
            timestep=0.1,
            render=True,
            sumo_map=None,
        ):
        super().__init__()
        self.render = render
        self.scenario_number = 0
        self.timestep = timestep
        if not sumo_map:
            raise SimulationCreationError("sumo_map needs to be specified")
        self.sumo_map = sumo_map
    
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
            **kwargs,
        )

    def destroy(self):
        super().destroy()


class MetaDriveSimulation(DrivingSimulation):
    def __init__(self, scene, render, scenario_number, timestep, sumo_map, **kwargs):
        if len(scene.objects) == 0:
            raise SimulationCreationError("The Metadrive interface requires you to define at least one Scenic object within the scene.")

        self.render = render
        self.scenario_number = scenario_number
        self.defined_ego = False
        self.client = None
        self.timestep = timestep
        self.sumo_map = sumo_map
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()
    
    def step(self):
        if (len(self.scene.objects) > 0):
            obj = self.scene.objects[0]
            action = obj.metaDriveActor.last_current_action[-1]
            o, r, tm, tc, info = self.client.step(action)

    def createObjectInSimulator(self, obj):
        if not self.defined_ego:
            self.client = DriveEnv(
                dict(
                    use_render=self.render,
                    vehicle_config={"spawn_position_heading": [utils.scenicToMetaDrivePosition(obj.position), obj.heading]},
                    use_mesh_terrain=True,
                    log_level=logging.CRITICAL,
                )
            )
            self.client.config['sumo_map'] = self.sumo_map
            self.client.reset()

            self.defined_ego = True

            metadrive_objects = self.client.engine.get_objects()
            for _,v in metadrive_objects.items():
                metaDriveActor = v
                obj.metaDriveActor = metaDriveActor
                return metaDriveActor
        
        if (type(obj).__name__ == "Car"):
            metaDriveActor = self.client.engine.spawn_object(DefaultVehicle, 
                                  vehicle_config=dict(), 
                                  position=utils.scenicToMetaDrivePosition(obj.position), 
                                  heading=obj.heading)    
            obj.metaDriveActor = metaDriveActor     
            
        return metaDriveActor

    def safe_clear_objects(self, object_ids):
        # Filter the list of object IDs to include only those that exist in the engine's spawned objects
        existing_object_ids = [obj_id for obj_id in object_ids if obj_id in self.client.engine._spawned_objects]
        breakpoint()
        # Call the clear_objects method with the filtered list
        self.client.engine.clear_objects(existing_object_ids, force_destroy=False)
    
    def destroy(self):
        if self.client:
            self.safe_clear_objects(list(self.client.engine._spawned_objects.keys()))
            # TODO: Clear only existing objects to avoid KeyError
            self.client.close()

        super().destroy()

    def getProperties(self, obj, properties):
        metaDriveActor = obj.metaDriveActor
        position = utils.metadriveToScenicPosition(metaDriveActor.last_position)
        velocity = utils.metadriveToScenicPosition(metaDriveActor.last_velocity)
        speed = metaDriveActor.last_speed
        angularSpeed=0
        angularVelocity=utils.metadriveToScenicPosition((0,0))
        yaw, pitch, _ = obj.parentOrientation.globalToLocalAngles(metaDriveActor.last_heading_dir[0], metaDriveActor.last_heading_dir[1], 0)

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


        
