# stub to allow changing the map without having to alter lgsvl_model.sc

import os.path

mapPath = None

def setMapPath(module, relpath):
    global mapPath
    base = os.path.dirname(module)
    mapPath = os.path.join(base, relpath)
