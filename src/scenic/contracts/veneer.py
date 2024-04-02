__all__ = (
    "scenarioFromFile",
    # Scenic+Contracts
    "BaseComponent",
    "ActionComponent",
    "ComposeComponent",
    "Contract",
    "SimulationTesting",
    "TimeTerminationCondition",
    "CountTerminationCondition",
    "GapTerminationCondition",
    "CorrectnessRequirementCondition",
    "runComponentsSimulation",
    "leadDistance",
    "registerVerifyStatement",
    "Assumption",
    "Composition",
)

# various Python types and functions used in the language but defined elsewhere
from scenic.contracts.testing import SimulationTesting
from scenic.contracts.utils import leadDistance, runComponentsSimulation
from scenic.syntax.translator import scenarioFromFile

# everything that should not be directly accessible from the language is imported here:
from scenic.contracts.assumption import Assumption
from scenic.contracts.components import ActionComponent, BaseComponent, ComposeComponent
from scenic.contracts.composition import Composition
from scenic.contracts.contracts import Contract
from scenic.contracts.testing import (
    CorrectnessRequirementCondition,
    CountTerminationCondition,
    GapTerminationCondition,
    TimeTerminationCondition,
)

### Internals
_verifyStatements = []
_syntaxTrees = None


def registerVerifyStatement(stmt):
    """Add a verify statement to the global tracker"""
    _verifyStatements.append(stmt)


def getSyntaxTrees():
    assert _syntaxTrees is not None
    return _syntaxTrees
