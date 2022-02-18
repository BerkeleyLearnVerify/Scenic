
"""Abstract interface to simulators supporting the driving domain."""

from scenic.core.simulators import Simulator, Simulation, GymSimulation

class DrivingSimulator(Simulator):
    def createSimulation(self, scene):
        raise NotImplementedError

class DrivingSimulation(Simulation):
    pass

class DrivingGymSimulation(GymSimulation):
    pass