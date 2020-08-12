"""A compiler and scene generator for the Scenic scenario description language.

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree: _autosummary

   core
   domains
   formats
   simulators
   syntax
"""

from .syntax.translator import scenarioFromFile, scenarioFromString
