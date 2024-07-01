from scenic.core.simulators import (
    Simulation,
    SimulationCreationError,
    Simulator,
    SimulatorInterfaceWarning,
)

from metadrive.envs import BaseEnv
import logging

class MetaDriveSimulator(Simulator):
    def __init__(self):
        self.client = BaseEnv(dict(use_render=False, # if you have a screen and OpenGL suppor, you can set use_render=True to use 3D rendering  
                manual_control=False, # we usually manually control the car to test environment
                log_level=logging.CRITICAL)) # suppress logging message
        super().__init__()
    
    def createSimulation(self, scene, **kwargs):
        simulation = MetaDriveSimulation(self, scene, self.client, **kwargs)
        self.client.top_down_renderer.generate_gif()
        return 

    def destroy(self):
        self.client.close()
        super().destroy()

class MetaDriveSimulation(Simulation):
    def __init__(self, simulator, scene, client, **kwargs):
        self.simulator = simulator
        self.client = client
        super().__init__(scene, **kwargs)
    
    def setup(self):
        super().setup()
    
    def step(self):
        self.client.step(self.client.action_space.sample())

        
