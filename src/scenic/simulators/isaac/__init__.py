"""Interface to the Isaac Sim and Isaac Lab robotics simulators.

This interface is experimental and subject to change.
Detailed documentation forthcoming; meanwhile, see :file:`examples/isaaclab/README.md`.
"""

from .simulator import IsaacSimSimulator, IsaacSimulator

try:
    from .lab import IsaacLabSimulator
except ModuleNotFoundError:
    IsaacLabSimulator = None


class TerrainBase:
    horizontalScale: float = 0.1
    verticalScale: float = 0.005
