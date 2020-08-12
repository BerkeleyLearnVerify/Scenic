"""Stub module to allow specifying the road network from a Scenic scenario."""

import os.path

from .roads import Network

network = None

def loadNetwork(path, **kwargs):
    global network
    network = Network.fromFile(path, **kwargs)

def loadLocalNetwork(module, relpath, **kwargs):
    base = os.path.dirname(module)
    path = os.path.join(base, relpath)
    loadNetwork(path, **kwargs)
