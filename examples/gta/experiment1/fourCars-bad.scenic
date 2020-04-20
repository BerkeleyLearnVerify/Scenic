from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param weather = 'RAIN'
param time = 0 * 60

wiggle = (-10 deg, 10 deg)

ego = EgoCar with roadDeviation wiggle
Car visible, with roadDeviation resample(wiggle)
Car visible, with roadDeviation resample(wiggle)
Car visible, with roadDeviation resample(wiggle)
Car visible, with roadDeviation resample(wiggle)