"""A compiler and scene generator for the Scenic scenario description language."""

from .syntax.translator import scenarioFromFile, scenarioFromString

import scenic.core.errors as _errors
_errors.showInternalBacktrace = False
del _errors
