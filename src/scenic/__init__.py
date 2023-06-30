"""A compiler and scene generator for the Scenic scenario description language."""

import scenic.core.errors as _errors
from scenic.core.errors import setDebuggingOptions
from scenic.syntax.translator import scenarioFromFile, scenarioFromString

_errors.showInternalBacktrace = False  # see comment in errors module
del _errors
