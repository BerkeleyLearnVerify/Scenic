from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param time = 12 * 60
param weather = 'EXTRASUNNY'

ego = EgoCar

c = Car on visible road,
	apparently facing 27.0516943340308 deg,
	with model CarModel.models['DOMINATOR'],
	with color CarColor.withBytes([187, 162, 157])