from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param time = 12 * 60
param weather = 'EXTRASUNNY'

ego = new EgoCar

c = new Car at ego offset by Range(-5, 5) @ Range(7, 12),
    with color Color.withBytes([187, 162, 157])