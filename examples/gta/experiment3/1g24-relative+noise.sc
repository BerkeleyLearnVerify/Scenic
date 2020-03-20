from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param time = 12 * 60
param weather = 'EXTRASUNNY'

ego = EgoCar

posOffset = (-625.4444493298472 - -628.78787878787944) @ (-530.76549003839568 - -540.60676779463461)
angleOffset = (8.287256822061408 deg - -359.16913666080427 deg) - 360 deg

c = Car offset by posOffset,
	facing angleOffset relative to ego.heading,
	with model CarModel.models['DOMINATOR'],
	with color CarColor.withBytes([187, 162, 157])

mutate