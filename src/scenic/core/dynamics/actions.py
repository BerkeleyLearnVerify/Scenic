"""Actions taken by dynamic agents."""

import abc


class Action(abc.ABC):
    """An :term:`action` which can be taken by an agent for one step of a simulation."""

    def canBeTakenBy(self, agent):
        """Whether this action is allowed to be taken by the given agent.

        The default implementation always returns True.
        """
        return True

    @abc.abstractmethod
    def applyTo(self, agent, simulation):
        """Apply this action to the given agent in the given simulation.

        This method should call simulator APIs so that the agent will take this action
        during the next simulated time step. Depending on the simulator API, it may be
        necessary to batch each agent's actions into a single update: in that case you
        can have this method set some state on the agent, then apply the actual update
        in an overridden implementation of `Simulation.executeActions`. For examples,
        see the CARLA interface: `scenic.simulators.carla.actions` has some CARLA-specific
        actions which directly call CARLA APIs, while the generic steering and braking
        actions from `scenic.domains.driving.actions` are implemented using the batching
        approach (see for example the ``setThrottle`` method of the class
        `scenic.simulators.carla.model.Vehicle`, which sets state later read by
        ``CarlaSimulation.executeActions`` in `scenic.simulators.carla.simulator`).
        """
        raise NotImplementedError


class _EndSimulationAction(Action):
    """Special action indicating it is time to end the simulation.

    Only for internal use.
    """

    def __init__(self, line):
        self.line = line

    def __str__(self):
        return f'"terminate simulation" executed on line {self.line}'

    def applyTo(self, agent, simulation):
        assert False


class _EndScenarioAction(Action):
    """Special action indicating it is time to end the current scenario.

    Only for internal use.
    """

    def __init__(self, scenario, line):
        self.scenario = scenario
        self.line = line

    def __str__(self):
        return f'"terminate" executed on line {self.line}'

    def applyTo(self, agent, simulation):
        assert False
