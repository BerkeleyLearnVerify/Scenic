"""Abstract interface to simulators supporting the driving domain."""

from scenic.core.simulators import Simulation, Simulator
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
    PurePursuitLateralController,
)


class DrivingSimulator(Simulator):
    """A `Simulator` supporting the driving domain."""

    pass


class DrivingSimulation(Simulation):
    """A `Simulation` with a simulator supporting the driving domain.

    This subclass of `Simulation` provides no special behavior by itself; it
    just provides convenience methods for creating controllers to be used by
    `FollowLaneBehavior` and related behaviors, so that the parameters of these
    controllers can be customized for different simulators.
    """

    def getLaneFollowingControllers(self, agent):
        """Get longitudinal and lateral controllers for lane following.

        The default controllers are simple PID controllers with parameters that
        work reasonably well for cars in simulators with realistic physics. See the
        classes `PIDLongitudinalController` and `PIDLateralController` for details,
        and `NewtonianSimulation` for an example of how to override this function.

        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller
    
    def getPurePursuitControllers(self, agent):
        dt = self.timestep

        #just assuming this is a car for development
        lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
        lat_controller = PurePursuitLateralController(dt = dt)

        return lon_controller, lat_controller
    
    def getTurningControllers(self, agent):
        """Get longitudinal and lateral controllers for turning."""
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.8, K_D=0.2, K_I=0.0, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.4, K_D=0.1, K_I=0.0, dt=dt)
        return lon_controller, lat_controller

    def getLaneChangingControllers(self, agent):
        """Get longitudinal and lateral controllers for lane changing."""
        dt = self.timestep
        if agent.isCar:
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.08, K_D=0.3, K_I=0.0, dt=dt)
        else:
            lon_controller = PIDLongitudinalController(
                K_P=0.25, K_D=0.025, K_I=0.0, dt=dt
            )
            lat_controller = PIDLateralController(K_P=0.1, K_D=0.3, K_I=0.0, dt=dt)
        return lon_controller, lat_controller
