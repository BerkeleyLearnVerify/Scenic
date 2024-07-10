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
import matplotlib.pyplot as plt
from scenic.core.simulators import SimulationCreationError
from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator
from scenic.domains.driving.roads import Network
from metadrive.engine.asset_loader import AssetLoader
from metadrive.engine.asset_loader import initialize_asset_loader


class MetaDriveSimulator(DrivingSimulator):
    def __init__(
            self,
            timestep=0.1,
            render=False,
            metadrive_map={},
        ):
        super().__init__()
        # check to see if metadrive map is active
        self.render = render

        self.client = DriveEnv(
            dict(
                agent_policy=IDMPolicy,
                use_render=self.render,
                # map_config=self.metadrive_map,
                traffic_density=0,
            )
        )
        o, _ = self.client.reset()

        if metadrive_map is not None:
            map_data = load_opendrive_map(metadrive_map)
            global_network = OpenDriveRoadNetwork()
            blocks = []
            for road in map_data.roads:
                for section in road.lanes.lane_sections:
                    block = OpenDriveBlock(len(blocks), global_network, 0, section)
                    block.construct_block(self.client.engine.render, self.client.engine.physics_world)
                    blocks.append(block)
            # import pdb; pdb.set_trace()
            # lanes = [lane_info.lane for lane_info in global_network.graph.values()]
            # for lane in lanes:
            #     import pdb; pdb.set_trace()
        else:
            self.metadrive_map={
                BaseMap.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_SEQUENCE, 
                BaseMap.GENERATE_CONFIG: "rCR",
                BaseMap.LANE_WIDTH: 3.5,
                BaseMap.LANE_NUM: 2
            }
        self.scenario_number = 0
        self.timestep = timestep
    
        # self.cleanup()  # Ensure cleanup before reset
        # import pdb;pdb.set_trace()
        # self.client.reset()
        # current_map = draw_top_down_map(self.client.current_map)
        # fig, ax = plt.subplots(figsize=(10, 10))  # Adjust figsize as needed
        # draw_top_down_map(current_map, ax)
        # ax.set_aspect('equal')  # Ensure aspect ratio is maintained
        # plt.show()
        self.client.agent_manager.reset()
        # self.client.reset()
    
    def createSimulation(self, scene, *, timestep, **kwargs):
        if timestep is not None and timestep != self.timestep:
            raise RuntimeError(
                "cannot customize timestep for individual MetaDrive simulations; "
                "set timestep when creating the MetaDriveSimulator instead"
            )
        self.scenario_number += 1 
        return MetaDriveSimulation(
            scene, 
            self.client, 
            self.render,
            self.scenario_number,
            timestep=self.timestep,
            **kwargs,
        )

    def destroy(self):
        self.client.close()
        super().destroy()


class MetaDriveSimulation(DrivingSimulation):
    def __init__(self, scene, client, render, scenario_number, **kwargs):
        self.client = client
        self.render = render
        self.scenario_number = scenario_number
        self.client = client
        super().__init__(scene, **kwargs)
    
    def setup(self):
        super().setup()
    
    def step(self):
        # import pdb; pdb.set_trace()
        # for i in range(1, 100000):
        # o, r, tm, tc, info = self.client.step(expert(self.client.agent))
        try:
            o, r, tm, tc, info = self.client.step([0,3])
            self.client.render(mode="top_down", screen_size=(500, 500),
                               screen_record=True,
                               window=True)
        except Exception as e:
            print(f"Error during step: {e}")
        # self.client.render(mode="top_down", text={"Quit": "ESC"}, film_size=(2000, 2000))

        
    def createObjectInSimulator(self, obj):
        import pdb; pdb.set_trace()
        if isinstance(obj.metaDriveActor, DefaultVehicle):
            metadriveActor = self.client.engine.spawn_object(DefaultVehicle, 
                                  vehicle_config=dict(), 
                                  position=(0, 0), 
                                  heading=0)
            # metadriveActor.setThrottle(0)
            
            
        return metadriveActor
        # if obj.rolename is not None:
        #     blueprint.set_attribute("role_name", obj.rolename)
        # return 
    
    def destroy(self):
        try:
            self.client.reset()
        except AssertionError as ae:
            print(f"Assertion error during reset: {ae}")
        
        super().destroy()

    def getProperties(self, obj, properties):
        return {}#super().getProperties(obj, properties)


        
