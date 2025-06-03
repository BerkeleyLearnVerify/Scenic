# src/scenic/simulators/robosuite/simulator.py
"""RoboSuite Simulator for Scenic."""

try:
    import robosuite as suite
    from robosuite.models import MujocoWorldBase
except ImportError as e:
    suite = None
    _import_error = e

from scenic.core.simulators import Simulation, Simulator
from scenic.core.vectors import Vector


class RobosuiteSimulator(Simulator):
    """Simulator interface for RoboSuite."""
    
    def __init__(self, render=True, real_time=True, speed=1.0):
        super().__init__()
        if suite is None:
            raise RuntimeError(f"Unable to import RoboSuite: {_import_error}")
        self.render = render
        self.real_time = real_time
        self.speed = speed
        
    def createSimulation(self, scene, **kwargs):
        return RobosuiteSimulation(scene, self.render, self.real_time, self.speed, **kwargs)


class RobosuiteSimulation(Simulation):
    """A simulation running in RoboSuite."""
    
    def __init__(self, scene, render, real_time, speed, **kwargs):
        self.render = render
        self.real_time = real_time
        self.speed = speed
        self.world = None
        self.model = None
        self.data = None
        self.viewer = None
        self._step_count = 0
        self._body_id_map = {}
        self._prev_positions = {}
        self._robots = []
        
        # Set timestep parameters
        self.timestep = kwargs.get('timestep') or 0.1
        self.physics_timestep = 0.002
        self.physics_steps = int(self.timestep / self.physics_timestep)
        
        super().__init__(scene, **kwargs)
        
    def destroy(self):
        """Clean up the simulation."""
        if self.viewer:
            try:
                self.viewer.close()
            except:
                pass
            self.viewer = None