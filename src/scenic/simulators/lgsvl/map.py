# stub to allow changing the map without having to alter lgsvl_model.sc

from scenic.simulators.domains.driving.network import loadNetwork, loadLocalNetwork

setMapPath = loadLocalNetwork   # for backwards compatibility

