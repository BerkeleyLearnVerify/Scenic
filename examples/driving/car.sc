from scenic.simulators.domains.driving.network import loadNetwork
loadNetwork('maps/cubetown.xodr')

from scenic.simulators.domains.driving.model import *

ego = Car