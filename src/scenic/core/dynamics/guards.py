"""Preconditions and invariants of behaviors and scenarios."""


class GuardViolation(Exception):
    """Abstract exception raised when a guard of a behavior is violated.

    This will never be raised directly; either of the subclasses `PreconditionViolation`
    or `InvariantViolation` will be used, as appropriate.
    """

    violationType = "guard"

    def __init__(self, behavior, lineno):
        self.behaviorName = behavior.__class__.__name__
        self.lineno = lineno

    def __str__(self):
        return (
            f"violated {self.violationType} of {self.behaviorName} on line {self.lineno}"
        )


class PreconditionViolation(GuardViolation):
    """Exception raised when a precondition is violated.

    Raised when a precondition is violated when invoking a behavior
    or when a precondition encounters a `RejectionException`, so that
    rejections count as precondition violations.
    """

    violationType = "precondition"


class InvariantViolation(GuardViolation):
    """Exception raised when an invariant is violated.

    Raised when an invariant is violated when invoking/resuming a behavior
    or when an invariant encounters a `RejectionException`, so that
    rejections count as invariant violations.
    """

    violationType = "invariant"
