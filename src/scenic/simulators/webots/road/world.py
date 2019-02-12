
# stub to allow changing the world without having to alter road_model.sc

import os.path

worldPath = None

def setLocalWorld(module, relpath):
    global worldPath
    base = os.path.dirname(module)
    worldPath = os.path.join(base, relpath)
