from .simulator import IsaacSimSimulator, IsaacSimulator

try:
    from .lab import IsaacLabSimulator
except ModuleNotFoundError:
    IsaacLabSimulator = None


class TerrainBase:
    horizontalScale: float = 0.1
    verticalScale: float = 0.005
