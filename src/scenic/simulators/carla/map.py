"""Stub to allow changing the map without having to change the model."""

import os.path

mapPath = None
lanePoints = 20

def setMapPath(module, relpath, points=None):
    global mapPath, lanePoints
    base = os.path.dirname(module)
    mapPath = os.path.join(base, relpath)
    if points is not None:
        lanePoints = points
