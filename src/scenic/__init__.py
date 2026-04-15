"""A compiler and scene generator for the Scenic scenario description language."""

import random as _random

import numpy as _numpy

import scenic.core.errors as _errors
from scenic.core.errors import setDebuggingOptions
from scenic.syntax.translator import scenarioFromFile, scenarioFromString

_errors.showInternalBacktrace = False  # see comment in errors module
del _errors


def setSeed(seed):
    """Seed the random number generators Scenic uses for sampling.

    Seeds both the Python :mod:`random` module and :mod:`numpy.random`, which
    are what Scenic's rejection sampler draws from. After calling this,
    subsequent calls into Scenic (e.g. :func:`scenarioFromFile` /
    :meth:`Scenario.generate`) will produce deterministic results for the
    given seed.

    This is the programmatic equivalent of the ``-s``/``--seed``
    command-line option; see :option:`--seed`.

    Args:
        seed (int): Seed value to pass to both RNGs.
    """
    _random.seed(seed)
    _numpy.random.seed(seed)
