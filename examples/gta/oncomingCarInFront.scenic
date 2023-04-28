from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car
c2 = new Car offset by Range(-10, 10) @ Range(20, 40), with viewAngle 30 deg
require c2 can see ego