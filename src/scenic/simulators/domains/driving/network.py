"""Stub module to allow specifying the road network from a Scenic scenario."""

from .roads import Network

network = None

def loadNetwork(path, **kwargs):
    global network
    network = Network.fromFile(path, **kwargs)
