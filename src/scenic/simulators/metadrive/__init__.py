"""Interface to the MetaDrive driving simulator.

This interface must currently be used in `2D compatibility mode`.

It supports dynamic scenarios involving vehicles and pedestrians.

The interface implements the :obj:`scenic.domains.driving` abstract domain, so any
object types, behaviors, utility functions, etc. from that domain may be used freely.
For details of additional MetaDrive-specific functionality, see the world model
:obj:`scenic.simulators.metadrive.model`.
"""

# Only import MetaDriveSimulator if the metadrive package is installed; otherwise the
# import would raise an exception.
metadrive = None
try:
    import metadrive
except ImportError:
    pass
if metadrive:
    from .simulator import MetaDriveSimulator
del metadrive
