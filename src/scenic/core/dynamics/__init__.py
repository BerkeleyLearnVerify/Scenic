"""Support for dynamic behaviors and modular scenarios.

A few classes are exposed here for external use, including:

* `Action`;
* `GuardViolation`, `InvariantViolation`, and `PreconditionViolation`;
* `StuckBehaviorWarning`.

Everything else defined in the submodules is an implementation detail and
should not be used outside of Scenic (it may change at any time).
"""

from .actions import Action
from .guards import GuardViolation, InvariantViolation, PreconditionViolation
from .utils import RejectSimulationException, StuckBehaviorWarning

#: Timeout in seconds after which a `StuckBehaviorWarning` will be raised.
stuckBehaviorWarningTimeout = 10
