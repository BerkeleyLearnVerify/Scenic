from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = Car
c2 = Car offset by Range(-10, 10) @ Range(20, 40),
	facing Range(-10 deg, 10 deg) relative to roadDirection

def angled(angle, epsilon=10 deg):
	a = apparent heading of c2
	return (abs(a - angle) <= epsilon) or (abs(a + angle) <= epsilon)

require angled(90 deg)

Car at c2 offset by Range(-10, 10) @ Range(-10, 10),
	facing Range(-10 deg, 10 deg) relative to roadDirection