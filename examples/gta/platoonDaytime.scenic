from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

param time = Range(8, 20) * 60	# 8 am to 8 pm

ego = Car with visibleDistance 60
c2 = Car visible
platoon = createPlatoonAt(c2, 5, dist=(2, 8))