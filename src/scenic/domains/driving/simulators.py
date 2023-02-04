
"""Abstract interface to simulators supporting the driving domain."""

from scenic.core.simulators import Simulator, Simulation

class DrivingSimulator(Simulator):
    def createSimulation(self, scene):
        raise NotImplementedError

class DrivingSimulation(Simulation):
    pass
