"""Stub to allow changing the map without having to change the model."""

import os.path

mapPath = None

def setMapPath(module, relpath):
    global mapPath
    base = os.path.dirname(module)
    mapPath = os.path.join(base, relpath)
