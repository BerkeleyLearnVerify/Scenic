from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

wiggle = (-10 deg, 10 deg)

ego = EgoCar with roadDeviation wiggle