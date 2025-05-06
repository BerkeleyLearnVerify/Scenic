from scenic.core.simulators import Simulation, Simulator
from scenic.simulators.metsr.simulator import METSRSimulator
from scenic.simulators.carla.simulator import CarlaSimulator
from scenic.core.vectors import Orientation, Vector

class CosimSimulator(Simulator):
    def __init__(self, map_name, carla_host, carla_port, metsr_host, metsr_port, timestep):
        super().__init__()

        breakpoint()

        self.map_name = map_name
        self.timestep = timestep
        self.sim_timestep = sim_timestep

        self.carla_sim = CarlaSimulator()
        self.metsr_sim = METSRSimulator()

    def createSimulation(self, scene, timestep, **kwargs):
        assert timestep is None or timestep == self.timestep

        return CosimSimulation(
            scene, self.timestep, self.carla_sim, self.metsr_sim, **kwargs
        )

    def destroy(self):
        self.carla_sim.destroy()
        self.metsr_sim.destroy()
        super().destroy()

class CosimSimulation(Simulation):
	pass