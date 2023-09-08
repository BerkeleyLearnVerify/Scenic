from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car
c2 = new Car at ego offset by Range(-5, 5) @ Range(20, 40)
new Car at c2 offset by Range(-10, 10) @ 0