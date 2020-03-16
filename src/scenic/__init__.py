"""A compiler and scene generator for the Scenic scenario description language.

Submodules
==========

.. autosummary::
   :toctree: _autosummary

   core
   simulators
   syntax
"""

from .syntax.translator import scenarioFromFile, scenarioFromString
