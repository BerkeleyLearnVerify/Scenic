from scenic.core.simulators import (
    Simulation,
    SimulationCreationError,
    Simulator,
    SimulatorInterfaceWarning,
)

from metadrive import TopDownMetaDrive
from metadrive.examples.ppo_expert.numpy_expert import expert
import random

class MetaDriveSimulator(Simulator):
    def __init__(self):
        self.client = TopDownMetaDrive(
        dict(
            # We also support using two renderer (Panda3D renderer and Pygame renderer) simultaneously. You can
            # try this by uncommenting next line.
            # use_render=True,

            # You can also try to uncomment next line with "use_render=True", so that you can control the ego vehicle
            # with keyboard in the main window.
            # manual_control=True,
            num_agents=1,
            map="SSSS",
            traffic_density=0,
            num_scenarios=1,
            # start_seed=random.randint(0, 1000),
            )
        )
        o, _ = self.client.reset()
        super().__init__()
    
    def createSimulation(self, scene, **kwargs):
        return MetaDriveSimulation(self, scene, self.client, **kwargs)

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
        # import pdb; pdb.set_trace()
        # for i in range(1, 100000):
        # o, r, tm, tc, info = self.client.step(expert(self.client.agent))
        o, r, tm, tc, info = self.client.step(expert(self.client.agent))
        self.client.render(mode="top_down", screen_size=(500, 500),
                   screen_record=True,
                   window=True)
        # self.client.render(mode="top_down", text={"Quit": "ESC"}, film_size=(2000, 2000))

        
    def createObjectInSimulator(self, obj):
        return #super().createObjectInSimulator(obj)
    
    def destroy(self):
        self.client.reset()
        super().destroy()

    def getProperties(self, obj, properties):
        return {}#super().getProperties(obj, properties)


        
