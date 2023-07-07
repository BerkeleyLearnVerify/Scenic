from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

ego = new Car
c2 = new Car visible
platoon = createPlatoonAt(c2, 5, dist=Range(2, 8))
start = platoon[0]
end = platoon[-1]
require abs(relative heading of end from start) >= 60 deg