from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = Car
c2 = Car offset by (-5, 5) @ (20, 40)
Car at c2 offset by (-10, 10) @ 0