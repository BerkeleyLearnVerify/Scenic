"""Stub to allow changing the CARLA map without changing the model."""

from scenic.domains.driving.network import loadNetwork, loadLocalNetwork

setMapPath = loadLocalNetwork   # for backwards compatibility
