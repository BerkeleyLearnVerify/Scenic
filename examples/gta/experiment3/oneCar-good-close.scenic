from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param weather = 'EXTRASUNNY'
param time = 12 * 60

wiggle = Range(-10 deg, 10 deg)

ego = EgoCar with roadDeviation wiggle
Car offset by Range(-5, 5) @ Range(7, 12), with roadDeviation resample(wiggle)