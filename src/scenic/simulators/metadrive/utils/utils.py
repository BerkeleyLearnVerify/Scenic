
from scenic.core.geometry import normalizeAngle
from scenic.core.vectors import Orientation, Vector

def metadriveToScenicPosition(loc):
    return Vector(loc[0], -loc[1], 0)
