"""Stub module to allow specifying the road network from a Scenic scenario."""

import os.path
import warnings

from .roads import Network

network = None

def loadNetwork(path, **kwargs):    # TODO this is for backwards compatibility; remove it
    global network
    if 'map' in globalParameters:
        warnings.warn('map already specified; ignoring call to loadNetwork')
    else:
        param map = path
        param map_options = kwargs
        network = Network.fromFile(path, **kwargs)

def loadLocalNetwork(module, relpath, **kwargs):
    base = os.path.dirname(module)
    path = os.path.join(base, relpath)
    loadNetwork(path, **kwargs)
