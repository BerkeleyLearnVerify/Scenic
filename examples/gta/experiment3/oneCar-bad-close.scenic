from scenic.simulators.gta.map import setLocalMap
setLocalMap(__file__, '../map.npz')

from scenic.simulators.gta.model import *

param weather = 'RAIN'
param time = 0 * 60

wiggle = Range(-10 deg, 10 deg)

ego = new EgoCar with roadDeviation wiggle
new Car at ego offset by Range(-5, 5) @ Range(7, 12), with roadDeviation resample(wiggle)