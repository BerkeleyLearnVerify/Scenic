"""A compiler and scene generator for the Scenic scenario description language."""

from scenic.syntax.translator import scenarioFromFile, scenarioFromString
from scenic.core.errors import setDebuggingOptions

import scenic.core.errors as _errors
_errors.showInternalBacktrace = False   # see comment in errors module
del _errors
