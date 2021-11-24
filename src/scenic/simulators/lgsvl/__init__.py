"""Interface to the LGSVL driving simulator.

This interface has been tested with `LGSVL <https://www.lgsvlsimulator.com/>`_ version
2020.06. It supports dynamic scenarios involving vehicles and pedestrians.

The interface implements the :obj:`scenic.domains.driving` abstract domain, so any
object types, behaviors, utility functions, etc. from that domain may be used freely.

.. raw:: html

   <h2>Submodules</h2>

.. autosummary::
   :toctree:

   model
   actions
   behaviors
   simulator
   utils
"""

# Only import LGSVLSimulator if the lgsvl package is installed; otherwise the
# import would raise an exception.
lgsvl = None
try:
   import lgsvl
except ImportError:
   pass
if lgsvl:
   from .simulator import LGSVLSimulator
del lgsvl
