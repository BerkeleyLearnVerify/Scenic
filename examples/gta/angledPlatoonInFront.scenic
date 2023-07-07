from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car
c2 = new Car at ego offset by Range(-10, 10) @ Range(20, 40),
    facing Range(-10 deg, 10 deg) relative to roadDirection

def angled(angle, epsilon=10 deg):
    a = apparent heading of c2
    return (abs(a - angle) <= epsilon) or (abs(a + angle) <= epsilon)

require angled(90 deg)

createPlatoonAt(c2, 2, dist=Range(2, 5))