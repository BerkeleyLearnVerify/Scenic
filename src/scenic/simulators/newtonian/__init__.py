"""Simple Newtonian physics simulator.

This interface must be used in `2D compatibility mode`.

This simulator allows dynamic scenarios to be tested without installing an
external simulator. It is currently very simplistic (e.g. not modeling
collisions).

The simulator provides two world models: a generic one, and a more specialized
model supporting traffic scenarios using the :obj:`scenic.domains.driving`
abstract domain.
"""

from .simulator import NewtonianSimulator
