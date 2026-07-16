from .simulator import IsaacSimSimulator, IsaacSimulator

try:
    from .lab import IsaacLabSimulator
except ModuleNotFoundError:
    IsaacLabSimulator = None


class TerrainBase:
    horizontal_scale: float = 0.1
    vertical_scale: float = 0.005
