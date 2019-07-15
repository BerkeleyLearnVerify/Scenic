
# stub to allow changing the world without having to alter road_model.sc

import os.path

mapPath = None

def setMapPath(module, relpath):
    global mapPath
    base = os.path.dirname(module)
    mapPath = os.path.join(base, relpath)
