"""Assorted utilities and classes used throughout the dynamics package."""


class RejectSimulationException(Exception):
    """Exception indicating a requirement was violated at runtime."""

    pass


class StuckBehaviorWarning(UserWarning):
    """Warning issued when a behavior/scenario may have gotten stuck.

    When a behavior or compose block of a modular scenario executes for a long
    time without yielding control, there is no way to tell whether it has
    entered an infinite loop with no take/wait statements, or is actually doing
    some long computation. But since forgetting a wait statement in a while loop
    is an easy mistake, we raise this warning after a behavior/scenario has run
    for `stuckBehaviorWarningTimeout` seconds without yielding.
    """

    pass
