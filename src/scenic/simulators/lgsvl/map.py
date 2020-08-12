# stub to allow changing the map without having to alter the model

from scenic.domains.driving.network import loadNetwork, loadLocalNetwork

setMapPath = loadLocalNetwork   # for backwards compatibility

