from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param weather = 'EXTRASUNNY'
param time = 12 * 60

wiggle = (-10 deg, 10 deg)

ego = EgoCar with roadDeviation wiggle
Car visible, with roadDeviation resample(wiggle)