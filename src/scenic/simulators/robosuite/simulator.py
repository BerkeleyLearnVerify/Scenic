"""
Simulator interface for the RoboSuite manipulation framework.
"""

import robosuite as suite

# Corrected the import for SimulationCreationError
from scenic.core.simulators import Simulator, Simulation, SimulationCreationError


class RobosuiteSimulator(Simulator):
    """A `Simulator` for the RoboSuite robotics framework."""
    def createSimulation(self, scene, **kwargs):
        """Create a new simulation from a Scenic scene."""
        return RobosuiteSimulation(scene, **kwargs)


class RobosuiteSimulation(Simulation):
    """A single simulation run in RoboSuite.

    Args:
        scene (Scene): The Scenic scene to simulate.
        **kwargs: Other arguments from `Simulator.simulate`.
    """
    def __init__(self, scene, **kwargs):
        # It's crucial to call the parent constructor FIRST.
        # This will set up all the necessary Scenic-internal state and
        # will call self.setup() for us.
        super().__init__(scene, **kwargs)

    def setup(self):
        """Set up the RoboSuite environment."""
        # This method is called by the parent __init__ after basic setup.
        # This is the correct place to create the simulator environment.
        super().setup() # This will call createObjectInSimulator for all objects

        # TODO: Extract environment and robot configuration from the Scenic scene.
        # For now, we use a hardcoded default for initial testing.
        env_name = 'Lift'
        robots = ['Panda']

        try:
            self.env = suite.make(
                env_name,
                robots=robots,
                has_renderer=True,            # Show the GUI for debugging
                has_offscreen_renderer=False,
                use_camera_obs=False,         # Do not need camera observations for now
                control_freq=20,              # Default control frequency
            )
            self.env.reset() # It's good practice to reset the env after creation
        except Exception as e:
            raise SimulationCreationError(f"Failed to create RoboSuite environment: {e}")


    def createObjectInSimulator(self, obj):
        """Create the given Scenic object in the RoboSuite simulation."""
        # This method will translate a Scenic object to a RoboSuite object.
        # For now, we do nothing as the environment is not yet scene-aware.
        pass

    def step(self):
        """Advance the simulation by one time step."""
        # Create a dummy action of the correct shape.
        action = [0.0] * self.env.action_spec[0].shape[0]
        self.env.step(action)
        # TODO: Update Scenic object properties from self.env.

    def getProperties(self, obj, properties):
        """Read the values of the given properties of an object from the simulator."""
        # This will be responsible for updating Scenic with the state from RoboSuite.
        raise NotImplementedError("getProperties has not been implemented yet.")

    def destroy(self):
        """Clean up after the simulation is over."""
        if hasattr(self, 'env') and self.env:
            self.env.close()
        super().destroy()