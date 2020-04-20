from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, 'map.npz')

from scenic.simulators.gta.model import *

param time = 16 * 60	# 4 pm
param weather = 'RAIN'

ego = EgoCar at -36 @ -491, facing -20 deg
c = Car offset by -3 @ 10, facing -40 deg,
	with model CarModel.models['BLISTA'],
	with color CarColor(1, 0, 0)
b = Bus offset by 1 @ 18, facing -22 deg,
	with color CarColor(0.9, 0.65, 0.2)

#mutate c
mutate