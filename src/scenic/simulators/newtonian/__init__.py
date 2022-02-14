"""Simple Newtonian physics simulator for traffic scenarios.

Allows scenarios written using the :obj:`scenic.domains.driving` abstract
domain to be simulated without installing an external simulator.

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree:

   model
   simulator
"""

from .simulator import NewtonianSimulator
