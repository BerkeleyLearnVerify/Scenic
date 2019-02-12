
# stub to allow changing the intersection without having to alter guideways_model.sc

import os.path

intersectionPath = None
intersectionOrigin = None

def setLocalIntersection(module, relpath):
    global intersectionPath
    base = os.path.dirname(module)
    intersectionPath = os.path.join(base, relpath)
