"""Simple Newtonian physics simulator.

This simulator allows dynamic scenarios to be tested without installing an
external simulator. It is currently very simplistic (e.g. not modeling
collisions).

The simulator provides two world models: a generic one, and a more specialized
model supporting traffic scenarios using the :obj:`scenic.domains.driving`
abstract domain.

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree:

   model
   driving_model
   simulator
"""

from .simulator import NewtonianSimulator
