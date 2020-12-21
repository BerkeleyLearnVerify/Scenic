"""Behaviors for dynamic agents in CARLA scenarios."""

from scenic.domains.driving.behaviors import *  # use common driving behaviors

try:
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    pass    # ignore; error will be caught later if user attempts to run a simulation

behavior WalkForwardBehavior():
    while True:
        take SetSpeedAction(0.5)

behavior AutopilotBehavior():
    """Behavior causing a vehicle to use CARLA's built-in autopilot."""
    take SetAutopilotAction(enabled=True)