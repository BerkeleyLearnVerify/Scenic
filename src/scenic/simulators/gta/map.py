
# stub to allow changing the map without having to alter gta_model.sc

import os

mapPath = 'map.npz'

def setLocalMap(module, relpath):
    global mapPath
    base = os.path.dirname(module)
    mapPath = os.path.join(base, relpath)
